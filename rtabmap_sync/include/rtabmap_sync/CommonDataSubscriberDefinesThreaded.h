/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// ** Threaded processing of input synced topics
// - In the previous implementation, synced topics WAITS to be processed until rtabmap's process() is finished.
// -- If the processing takes long, the backlog of input data grows, leading to pileup of unprocessed data.
// -- This leads to tf problems (tf listener only holds topic for certain duration).
// -- This also makes rtabmap_ros think that it did not receive input data from ROS topics.
// -- This happens because the callback of the synchronizer (message_filters::Synchronizer) leads to a call to CoreWrapper::process(), which is all done in a single thread.
// -- This patch aims to make the callback for the synchronizer run in a different thread from the processing, so that this callback has minimal processing time.
// -- This fix also allows implementation of threading on any callbacks, WITHOUT NEEDING TO MODIFY THEM, so that original implementation is preserved.

// - THIS WILL MAKE RTABMAP_ROS IGNORE INPUT DATA IN BETWEEN ITS PROCESSING TIMES.

// - As much as possible, this fix is implemented in a header, with minimal code changes on cpp files.
// -- CommonDataSubscriberDefinesThreaded.h is included in CommonDataSubscriber.h, so all code that uses CommonDataSubscriber.h can easily have access to the threaded version.

// - To implement this, you have to:
// -- 1) Include CommonDataSubscriberDefinesThreaded.h
// -- 2) Replace the declaration macros DATA_SYNCSx() with DATA_SYNCSx_THREADED() in CommonDataSubscriber.h (or where ever it is needed). e.g. DATA_SYNCS4 to DATA_SYNCS4_THREADED
// --- 2.1) Note: This is already done in CommonDataSubscriber.h
// -- 3) Replace the callback macros SYNC_DECLx() with callback setup macros SYNC_DECLx_THREADED() macro in implementation cpp's like rtabmap_sync/src/impl/CommonDataSubscriberDepth.cpp
// --- 3.1) Note: This is already done for CommonDataSubscriberDepth.cpp

#ifndef INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_H_
// We need definitions inside <rtabmap_sync/CommonDataSubscriberDefines.h>
#include <rtabmap_sync/CommonDataSubscriberDefines.h>
#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_H_ */


#ifndef INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_THREADED_H_
#define INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_THREADED_H_

#include <boost/thread.hpp>
#include <functional>
#include <type_traits>
#include <variant>


// **************************************
// Templates for finding index of a type in a parameter pack, __::value == -1 if type is not found
// Example usage: int idxLaser = find_first_type<0, sensor_msgs::LaserScanConstPtr, Args...>::value;

// template <typename What>
// bool constexpr containsType() {
// 	return false;
// }

// template <typename What, typename A, typename... Tail>
// bool constexpr containsType() {
// 	fprintf(stderr, "^ Compare [%s] and [%s]: %d...\n", typeid(What).name(), typeid(A).name(), std::is_same<What, std::remove_cv_t<std::remove_reference_t<A>>>::value);
// 	bool retval = std::is_same<What, std::remove_cv<std::remove_reference<A>>>::value ? true : containsType<What, Tail...>();
// 	return retval;
// }

// DESCRIPTION: This template will find the index TYPE_TO_FIND in the parameter pack PARAMETER_TYPES... in COMPILE TIME.
//				This will ignore constant, volatile, and reference qualifiers.
//				This will have a value of -1 when TYPE_TO_FIND is not found in the PARAMETER_TYPES...
// USAGE:	To get integer index: find_first_type<INDEX_COUNT_START, TYPE_TO_FIND, PARAMETER_TYPES...>::value
// NOTES:	INDEX_COUNT_START should always be 0.
//			TYPE_TO_FIND 
// EXAMPLE: find_first_type<0, sensor_msgs::LaserScanConstPtr, Args...>::value

#define DECLARE_INDEX_SEARCH_FUNC() \
	template <int Index, typename What, typename... Rest> \
	struct find_first_type;  \
	 \
	/* Last type is checked here. This is for general, NOT a template specialization. */ \
	/* This will be called when True, a specialization is made separately for false */ \
	template<int Index, bool IsValid> \
	struct find_first_type_end \
		: std::integral_constant<int, Index> {}; \
	 \
	/* When the last type is not the same as [What], a specialization for False.*/ \
	template<int Index> \
	struct find_first_type_end<Index, false> \
	    : std::integral_constant<int, -1> {}; \
	 \
	/* When there is only the Head remaining to check*/ \
	template<int Index, typename What, typename Head> \
	struct find_first_type<Index, What, Head> \
		: find_first_type_end<Index, std::is_same	< 	std::remove_cv_t<std::remove_reference_t<What>>,  \
														std::remove_cv_t<std::remove_reference_t<Head>>  \
													>::value > {}; \
	 \
	/* When there are at least 2 elements to check (1st to n-1 type), this gets nested.*/ \
	template<int Index, typename What, typename Head, typename... Rest> \
	struct find_first_type<Index, What, Head, Rest...> \
		: std::conditional_t<   std::is_same<std::remove_cv_t<std::remove_reference_t<What>>, std::remove_cv_t<std::remove_reference_t<Head>> >::value, \
							    std::integral_constant<int, Index>, \
							    find_first_type<Index+1, What, Rest...>> {}; \


// *********
// Determine source/index of the stamp to be used for tick. We only need to get stamp from available data.
// Order of precedence: LaserScan > PointCloud > CameraInfo > ZERO (ros::Time data_stamp; // default ctor)
// Ref1: CoreWrapper::commonMultiCameraCallback():1207 ... continue to:
// Ref2: CoreWrapper::odomUpdate() in CoreWrapper.cpp:1023
// NOTE:	Instead of returning -1 (no suitable source of stamp), just return the 1st argument, index 0.
//			We only need this stamp to trigger the tick of sync diagnostic, 
//			so that warnings will not trigger if the processing takes too long.
#define DECLARE_IDX_SEARCH_STAMP() \
	template<typename... Args> \
	constexpr static int getArgIndexForStamp() \
	{ \
		using idxLaser = find_first_type<0, sensor_msgs::LaserScanConstPtr, Args...>; \
		using idxPCL2 = find_first_type<0, sensor_msgs::PointCloud2ConstPtr, Args...>; \
		using idxCamInfo = find_first_type<0, sensor_msgs::CameraInfoConstPtr, Args...>; \
		 \
		if(idxLaser::value != -1) \
		{ \
			return idxLaser::value; \
		} \
		else if(idxPCL2::value != -1) \
		{ \
			return idxPCL2::value; \
		} \
		else if(idxCamInfo::value != -1) \
		{ \
			return idxCamInfo::value; \
		} \
		 \
		return 0; \
	}; \


// *********
// Extract the stamp from the arguments, by using the argument index got from getArgIndexForStamp(). Store it in the given stamp.
// This template is an if constexpr() alternative for compilers with lower that c++17, calculations are done in compile time.
#define DECLARE_GET_STAMP_FOR_TICK() \
	template <int idx, typename BoolType> \
	struct dummy; \
	 \
	template <int idx> \
	struct dummy <idx,  std::integral_constant<bool, true>::type> { \
		template<typename... Args> \
		static void MyFunc(ros::Time& data_stamp, Args&&... args) { \
			data_stamp = std::get<idx>(std::forward_as_tuple(args...))->header.stamp; \
			/*ROS_WARN("-----positive: %d, %f", idx, data_stamp.toSec());*/ \
		}; \
	}; \
	 \
	template <int idx> \
	struct dummy <idx,  std::integral_constant<bool, false>::type> \
	{ \
		template<typename... Args> \
		static void MyFunc(ros::Time& data_stamp, Args&&... args) { /*ROS_WARN("------ Negative:");*/ }; \
	}; \
	 \
	template<int idx, typename... Args> \
	static void getStampForTick(ros::Time& stamp, Args&&... args) \
	{ \
		dummy<idx, std::conditional_t<idx != -1, std::integral_constant<bool,true>::type, std::integral_constant<bool,false>::type > >::MyFunc(stamp, args...); \
	}; \
	 \
	/*template<typename Arg> \
	static void printros(const Arg& arg) \
	{ \
		ROS_WARN("**TIME %s: %f", typeid(Arg).name(), arg->header.stamp.toSec()); \
	};*/ \



// NOTE:	INLINE! This should be used inside DATA_SYNCSX_THREADED macro to create a proper thread spawner function like depthScan3dThreadSpawnerCallback()
// 			The function that contains this genericThreadSpawnerCallback() SHOULD STILL run on message_filters' thread.
// NOTE2:	STATIC! This is static and this template gets a pointer to instance of the caller.
//			This is done so that PREFIX##ThreadSpawnerCallback() (which calls this genericThreadSpawnerCallback()) inside
//			DATA_SYNCSx_THREADED() macro can give a context to this static function.
//			The end result is that you don't have to re-code genericThreadSpawnerCallback() in other classes that wants to implement this.
//			***BUT you should still declare the thread-related vars (dpThreadRunning_ and threadDataProcessing_) in those classes.
#define DECLARE_GENERIC_THREAD_SPAWNER() \
	template<typename ClassOfCaller, typename... Args> \
	inline static void genericThreadSpawnerCallback(ClassOfCaller *class_instance, const std::function<void()> & boundCB, Args&&... args) \
	{ \
		/* ROS_WARN("########### genericThreadSpawnerCallback."); */ \
		/*constexpr int idxLaser = find_first_type<0, sensor_msgs::LaserScanConstPtr, Args...>::value; \
		constexpr int idxPCL2 = find_first_type<0, sensor_msgs::PointCloud2ConstPtr, Args...>::value; \
		constexpr int idxCamInfo = find_first_type<0, sensor_msgs::CameraInfoConstPtr, Args...>::value; \
		ROS_WARN("** LASERSCAN %s [%d]", idxLaser==-1 ? "does NOT exist " : "DO exist", idxLaser); \
		ROS_WARN("** PCL2 %s [%d]", idxPCL2==-1 ? "does NOT exist " : "DO exist", idxPCL2); \
		ROS_WARN("** CAMINFO %s [%d]", idxCamInfo==-1 ? "does NOT exist " : "DO exist", idxCamInfo);*/ \
		 \
		ros::Time data_stamp; \
		constexpr int idx_for_stamp = getArgIndexForStamp<Args...>(); \
		getStampForTick<idx_for_stamp, Args...>(data_stamp, args...); \
		 \
		/* ROS_WARN("** Index for stamp: %d", idx_for_stamp); \
		ROS_WARN("** TIME for stamp: %f", data_stamp.toSec() ); \
		ROS_WARN("** TIME now: %f", ros::Time::now().toSec()); */ \
		 \
		/*using expander = int[]; \
		(void)expander{0, ((void)printros(std::forward<Args>(args)), 0)...};*/ \
		 \
		/*double tnow = ros::Time::now().toSec(); \
		double tdata = data_stamp.toSec(); \
		double tdiff = tnow - tdata;*/ \
		 \
		/* check if threadProcessData is running*/ \
		if (class_instance->dpThreadRunning_ == true) \
		{	/* thread is running, just tick*/ \
			/* class_instance->tick(data_stamp, 0.0);*/ \
			/* ROS_WARN("########### TICKING."); */ \
			if(class_instance->syncDiagnostic_.get()) \
			{ \
				class_instance->syncDiagnostic_->tick(data_stamp, 0.0); \
			} \
			/*ROS_WARN("** Skipped processing: tnow, diff =<%f, %f>", tnow, tdiff); \
			ROS_WARN("** Prev thread still running [%s].\n*** Will not use data from synchronizer... Exiting depthScan3dCallback()\n############\n", (boost::lexical_cast<std::string>(class_instance->threadDataProcessing_->get_id())).c_str());*/ \
		}  \
		\
		if (class_instance->dpThreadRunning_ == false) \
		{ \
			/*ROS_WARN("** Prev thread is NOT running.");*/ \
			if(class_instance->threadDataProcessing_ != 0) \
			{ \
				/* ROS_WARN("*** Prev thread is NOT NULL, check if joinable..."); */ \
				if(class_instance->threadDataProcessing_->joinable()){ \
					/* ROS_WARN("**** Prev thread is joinable..."); */ \
					class_instance->threadDataProcessing_->join(); \
					delete class_instance->threadDataProcessing_; \
					class_instance->threadDataProcessing_ = nullptr; \
					/* ROS_WARN("***** Just joined prev thread."); */ \
				} \
			} \
			/* ROS_WARN("*** SPAWNING new thread for data.. \n"); \
			ROS_WARN("*** Processed data: tnow, diff =<%f, %f>\n", tnow, tdiff); */ \
			class_instance->dpThreadRunning_ = true; \
			class_instance->threadDataProcessing_ = new boost::thread(boundCB);	/*// boundCB should be std::bind(CB, this, args...)*/ \
			/* ROS_WARN("**** Spawned new processing thread [%s]... Exiting genericThreadSpawnerCallback()\n############\n", (boost::lexical_cast<std::string>(class_instance->threadDataProcessing_->get_id())).c_str()); */ \
		} \
	}; \















// A threaded version of the callbacks that message_filters call after producing a synced set of messages.
// This is designed such that we can use the threaded version the original callbacks (like in CommonDataSubscriberDepth.cpp)
// 	without touching/affecting the original callback's implementation.
//
// From:
// [mesage_filters] -> [origCallback]
// To:
// [mesage_filters] -> [threadSpawnerCallback]
//								||
//								\/
//						[wrapperCallback] -> [origCallback]
//
// threadSpawnerCallback - 	will initiate creation of new thread in which wrapperCallback will be called. Implemented as genericThreadSpawnerCallback()
// wrapperCallback - 	is a wrapper for the original callback to facilitate unsetting 
//						dpThreadRunning_=false (flag that the thread has finished).
//						Implemented by macros DATA_SYNCSx_THREADED as PREFIX##ThreadedWrapperCallback() [e.g. depthScan3dThreadedWrapperCallback()]
// origCallback - 	the ORIGINAL implementation of the callback (like depthScan3dCallback(), 
//					which is declared (inside CommonDataSubscriber.h) with the macro DATA_SYNCS4, 
//					and setup (inside CommonDataSubscriber::setupDepthCallbacks()) in the macro SYNC_DECL4)



// ******************************************************
// ***** THREAD-RELATED VARIABLES DECLARATION:

#define DECLARE_THREADING_VARIABLES() \
		boost::thread* threadDataProcessing_ = nullptr; \
		bool dpThreadRunning_ = false; \
		DECLARE_INDEX_SEARCH_FUNC() \
		DECLARE_IDX_SEARCH_STAMP() \
		DECLARE_GET_STAMP_FOR_TICK() \
		DECLARE_GENERIC_THREAD_SPAWNER() \

// Put this macro at end of the object's destructor. 
#define WAIT_FOR_THREAD_AT_DESTRUCTOR() \
	if(threadDataProcessing_ != nullptr) \
	{ \
		/*std::cout << "******* A THREAD IS STILL RUNNING ***" << std::endl;*/ \
		if(threadDataProcessing_->joinable()) \
		{ \
			threadDataProcessing_->join(); \
			dpThreadRunning_ = true; /* stop spawning new threads */ \
			/*std::cout << "******* STOPPED RUNNING THREAD ***" << std::endl;*/ \
		} \
	} \
	delete threadDataProcessing_; \



// ******************************************************
// ***** DECLARATION MACROS: 	Should be invoked inside the class you are using in (e.g. CommonDataSubscriber).

// std::bind() in  void PREFIX##ThreadSpawnerCallback() is intentionally NOT USING boost::cref for the arguments
//	to make std::bind() make copies of the pointers it receives, so that the new thread will take ownership of the shared pointers

#define DATA_SYNCS2_THREADED(PREFIX, MSG0, MSG1) \
		DATA_SYNC2(PREFIX, Approximate, MSG0, MSG1) \
		DATA_SYNC2(PREFIX, Exact, MSG0, MSG1) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b) \
		{ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b); \
			if(syncDiagnostic_.get()) {syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0);} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b) \
		{ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b); \
			genericThreadSpawnerCallback(this, fp, a, b); \
		}; \

#define DATA_SYNCS3_THREADED(PREFIX, MSG0, MSG1, MSG2) \
		DATA_SYNC3(PREFIX, Approximate, MSG0, MSG1, MSG2) \
		DATA_SYNC3(PREFIX, Exact, MSG0, MSG1, MSG2) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&, const MSG2##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b, MSG2##ConstPtr c) \
		{ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b, c); \
			if(syncDiagnostic_.get()) {syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0);} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b, const MSG2##ConstPtr& c) \
		{ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b, c); \
			genericThreadSpawnerCallback(this, fp, a, b, c); \
		}; \

#define DATA_SYNCS4_THREADED(PREFIX, MSG0, MSG1, MSG2, MSG3) \
		DATA_SYNC4(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3) \
		DATA_SYNC4(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&, const MSG2##ConstPtr&, const MSG3##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b, MSG2##ConstPtr c, MSG3##ConstPtr d) \
		{ \
			/* ROS_WARN("@@@@@ calling original callback.."); */ \
			/*ROS_WARN("!!!!! newthread MSG0 use_count: %ld ", a.use_count());*/ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b, c, d); \
			/* ROS_WARN("@@@@@ exiting the wrapper for the original callback.. \n"); */ \
			if(syncDiagnostic_.get()) /* If the callback did NOT tick, we call tick here, just use the 1st argument as stamp source */ \
			{ \
				syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0); \
			} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b, const MSG2##ConstPtr& c, const MSG3##ConstPtr& d) \
		{ \
			/*ROS_WARN("!!!!! oldthread MSG0 use_count: %ld", a.use_count());*/ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b, c, d); \
			genericThreadSpawnerCallback(this, fp, a, b, c, d); \
		}; \

#define DATA_SYNCS5_THREADED(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4) \
		DATA_SYNC5(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4) \
		DATA_SYNC5(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&, const MSG2##ConstPtr&, const MSG3##ConstPtr&, const MSG4##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b, MSG2##ConstPtr c, MSG3##ConstPtr d, MSG4##ConstPtr e) \
		{ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b, c, d, e); \
			if(syncDiagnostic_.get()) {syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0);} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b, const MSG2##ConstPtr& c, const MSG3##ConstPtr& d, const MSG4##ConstPtr& e) \
		{ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b, c, d, e); \
			genericThreadSpawnerCallback(this, fp, a, b, c, d, e); \
		}; \

#define DATA_SYNCS6_THREADED(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		DATA_SYNC6(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		DATA_SYNC6(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&, const MSG2##ConstPtr&, const MSG3##ConstPtr&, const MSG4##ConstPtr&, const MSG5##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b, MSG2##ConstPtr c, MSG3##ConstPtr d, MSG4##ConstPtr e, MSG5##ConstPtr f) \
		{ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b, c, d, e, f); \
			if(syncDiagnostic_.get()) {syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0);} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b, const MSG2##ConstPtr& c, const MSG3##ConstPtr& d, const MSG4##ConstPtr& e, const MSG5##ConstPtr& f) \
		{ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b, c, d, e, f); \
			genericThreadSpawnerCallback(this, fp, a, b, c, d, e, f); \
		}; \

#define DATA_SYNCS7_THREADED(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		DATA_SYNC7(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		DATA_SYNC7(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&, const MSG2##ConstPtr&, const MSG3##ConstPtr&, const MSG4##ConstPtr&, const MSG5##ConstPtr&, const MSG6##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b, MSG2##ConstPtr c, MSG3##ConstPtr d, MSG4##ConstPtr e, MSG5##ConstPtr f, MSG6##ConstPtr g) \
		{ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b, c, d, e, f, g); \
			if(syncDiagnostic_.get()) {syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0);} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b, const MSG2##ConstPtr& c, const MSG3##ConstPtr& d, const MSG4##ConstPtr& e, const MSG5##ConstPtr& f, const MSG6##ConstPtr& g) \
		{ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b, c, d, e, f, g); \
			genericThreadSpawnerCallback(this, fp, a, b, c, d, e, f, g); \
		}; \

#define DATA_SYNCS8_THREADED(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		DATA_SYNC8(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		DATA_SYNC8(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		void PREFIX##Callback(const MSG0##ConstPtr&, const MSG1##ConstPtr&, const MSG2##ConstPtr&, const MSG3##ConstPtr&, const MSG4##ConstPtr&, const MSG5##ConstPtr&, const MSG6##ConstPtr&, const MSG7##ConstPtr&); \
		void PREFIX##ThreadedWrapperCallback(MSG0##ConstPtr a, MSG1##ConstPtr b, MSG2##ConstPtr c, MSG3##ConstPtr d, MSG4##ConstPtr e, MSG5##ConstPtr f, MSG6##ConstPtr g, MSG7##ConstPtr h) \
		{ \
			syncDiagnostic_->resetRecentlyTickedFlag(); \
			PREFIX##Callback(a, b, c, d, e, f, g, h); \
			if(syncDiagnostic_.get()) {syncDiagnostic_->tickIfNotRecentlyTicked(a->header.stamp, 0.0);} \
			dpThreadRunning_ = false; \
		}; \
		template <typename CallerClass> \
		void PREFIX##ThreadSpawnerCallback(const MSG0##ConstPtr& a, const MSG1##ConstPtr& b, const MSG2##ConstPtr& c, const MSG3##ConstPtr& d, const MSG4##ConstPtr& e, const MSG5##ConstPtr& f, const MSG6##ConstPtr& g, const MSG7##ConstPtr&h) \
		{ \
			auto fp = std::bind(&CallerClass::PREFIX##ThreadedWrapperCallback, this, a, b, c, d, e, f, g, h); \
			genericThreadSpawnerCallback(this, fp, a, b, c, d, e, f, g, h); \
		}; \



// ******************************************************
// ***** CALLBACK SETUP MACROS:		Setup for the callback that is needed by message_filters::Synchronizer (approx or exact).
// *****							Should be called in implementation of setupXXXCallbacks() (e.g. in CommonDataSubscriberDepth.cpp CommonDataSubscriber::setupDepthCallbacks())
// *****							The only difference with non-threaded version is these macros calls PREFIX##ThreadSpawnerCallback() instead of PREFIX##Callback()

#define SYNC_DECL2_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str());

#define SYNC_DECL3_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str(), \
				SUB2.getTopic().c_str());

#define SYNC_DECL4_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str(), \
				SUB2.getTopic().c_str(), \
				SUB3.getTopic().c_str());

#define SYNC_DECL5_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str(), \
				SUB2.getTopic().c_str(), \
				SUB3.getTopic().c_str(), \
				SUB4.getTopic().c_str());

#define SYNC_DECL6_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4, SUB5) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str(), \
				SUB2.getTopic().c_str(), \
				SUB3.getTopic().c_str(), \
				SUB4.getTopic().c_str(), \
				SUB5.getTopic().c_str());

#define SYNC_DECL7_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6, boost::placeholders::_7)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6, boost::placeholders::_7)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str(), \
				SUB2.getTopic().c_str(), \
				SUB3.getTopic().c_str(), \
				SUB4.getTopic().c_str(), \
				SUB5.getTopic().c_str(), \
				SUB6.getTopic().c_str());

#define SYNC_DECL8_THREADED(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6, SUB7) \
		fprintf(stderr, "\n\n\n\n USING prefix: [" #PREFIX "Callback]\n\n\n\n\n"); \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6, SUB7); \
			PREFIX##ApproximateSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6, boost::placeholders::_7, boost::placeholders::_8)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6, SUB7); \
			PREFIX##ExactSync_->registerCallback(boost::bind(&CLASS::PREFIX##ThreadSpawnerCallback<CLASS>, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6, boost::placeholders::_7, boost::placeholders::_8)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				SUB0.getTopic().c_str(), \
				SUB1.getTopic().c_str(), \
				SUB2.getTopic().c_str(), \
				SUB3.getTopic().c_str(), \
				SUB4.getTopic().c_str(), \
				SUB5.getTopic().c_str(), \
				SUB6.getTopic().c_str(), \
				SUB7.getTopic().c_str());



#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_THREADED_H_ */
