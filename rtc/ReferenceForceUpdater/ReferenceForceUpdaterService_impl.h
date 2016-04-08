// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __REFERENCEFORCEUPDATOR_SERVICE_H__
#define __REFERENCEFORCEUPDATOR_SERVICE_H__

#include "hrpsys/idl/ReferenceForceUpdaterService.hh"

class ReferenceForceUpdaterService_impl
	: public virtual POA_OpenHRP::ReferenceForceUpdaterService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	ReferenceForceUpdaterService_impl();

	/**
	   \brief destructor
	*/
	virtual ~ReferenceForceUpdaterService_impl();

	void echo(const char *msg);
private:
};

#endif
