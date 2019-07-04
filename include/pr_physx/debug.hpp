#ifndef __PR_PX_DEBUG_HPP__
#define __PR_PX_DEBUG_HPP__

#include "common.hpp"
#include <pragma/physics/visual_debugger.hpp>
#include <pvd/PxPvd.h>
#if 0
struct MyPvdClient : public physx::pvdsdk::PvdClient
{
	virtual void onPvdConnected()
	{
		// 1. create a PvdDataStream
		// 2. send your custom PVD class descriptions from here
		// this then allows PVD to correctly identify and represent
		// custom data that is sent from your application to a PxVisualDebuggerConnection.
		// example in JointConnectionHandler
		// 3. do something when successfully connected
		// e.g. enable contact and constraint visualization
	}
	virtual void onPvdDisconnected()
	{
		// handle disconnection, release PvdDataStream
		// e.g. disable contact and constraint visualization
	}
	//impleament other methods
	...
};

// register custom handler
MyPvdClient myPvdClient;
pvd->addClient(myPvdClient);
#endif
namespace pragma::physics
{
	class PxVisualDebugger
		: public IVisualDebugger
	{
	public:
	};
};

#endif
