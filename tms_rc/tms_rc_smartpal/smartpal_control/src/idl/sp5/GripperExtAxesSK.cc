// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "GripperExtAxes.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_2;



GripperExtAxes_ptr GripperExtAxes_Helper::_nil() {
  return ::GripperExtAxes::_nil();
}

::CORBA::Boolean GripperExtAxes_Helper::is_nil(::GripperExtAxes_ptr p) {
  return ::CORBA::is_nil(p);

}

void GripperExtAxes_Helper::release(::GripperExtAxes_ptr p) {
  ::CORBA::release(p);
}

void GripperExtAxes_Helper::marshalObjRef(::GripperExtAxes_ptr obj, cdrStream& s) {
  ::GripperExtAxes::_marshalObjRef(obj, s);
}

GripperExtAxes_ptr GripperExtAxes_Helper::unmarshalObjRef(cdrStream& s) {
  return ::GripperExtAxes::_unmarshalObjRef(s);
}

void GripperExtAxes_Helper::duplicate(::GripperExtAxes_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

GripperExtAxes_ptr
GripperExtAxes::_duplicate(::GripperExtAxes_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

GripperExtAxes_ptr
GripperExtAxes::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


GripperExtAxes_ptr
GripperExtAxes::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

GripperExtAxes_ptr
GripperExtAxes::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_GripperExtAxes _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_GripperExtAxes* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_GripperExtAxes;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* GripperExtAxes::_PD_repoId = "IDL:GripperExtAxes:1.0";


_objref_GripperExtAxes::~_objref_GripperExtAxes() {
  
}


_objref_GripperExtAxes::_objref_GripperExtAxes(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::GripperExtAxes::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
_objref_GripperExtAxes::_ptrToObjRef(const char* id)
{
  if( id == ::GripperExtAxes::_PD_repoId )
    return (::GripperExtAxes_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::GripperExtAxes::_PD_repoId) )
    return (::GripperExtAxes_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  _cboolean
class _0RL_cd_dc4492deb8062559_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_dc4492deb8062559_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::Boolean result;
};

void _0RL_cd_dc4492deb8062559_00000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalBoolean(result);

}

void _0RL_cd_dc4492deb8062559_00000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalBoolean();

}

const char* const _0RL_cd_dc4492deb8062559_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_10000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->servoOn();


}

::CORBA::Boolean _objref_GripperExtAxes::servoOn()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_10000000, "servoOn", 8);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_20000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->servoOff();


}

::CORBA::Boolean _objref_GripperExtAxes::servoOff()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_20000000, "servoOff", 9);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_30000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->isPowerOn();


}

::CORBA::Boolean _objref_GripperExtAxes::isPowerOn()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_30000000, "isPowerOn", 10);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_40000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->isServoOn();


}

::CORBA::Boolean _objref_GripperExtAxes::isServoOn()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_40000000, "isServoOn", 10);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_50000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->pause();


}

::CORBA::Boolean _objref_GripperExtAxes::pause()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_50000000, "pause", 6);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_60000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->resume();


}

::CORBA::Boolean _objref_GripperExtAxes::resume()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_60000000, "resume", 7);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_70000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->stop();


}

::CORBA::Boolean _objref_GripperExtAxes::stop()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_70000000, "stop", 5);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_80000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->abort();


}

::CORBA::Boolean _objref_GripperExtAxes::abort()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_80000000, "abort", 6);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_90000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->isMoving();


}

::CORBA::Boolean _objref_GripperExtAxes::isMoving()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_90000000, "isMoving", 9);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_a0000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_00000000* tcd = (_0RL_cd_dc4492deb8062559_00000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->clearAlarms();


}

::CORBA::Boolean _objref_GripperExtAxes::clearAlarms()
{
  _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_a0000000, "clearAlarms", 12);


  _invoke(_call_desc);
  return _call_desc.result;


}
// Proxy call descriptor class. Mangled signature:
//  _cboolean_i_cunsigned_plong_o_cunsigned_plong_o_cAlarmSeq
class _0RL_cd_dc4492deb8062559_b0000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_dc4492deb8062559_b0000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::ULong arg_0;
  ::CORBA::ULong arg_1;
  AlarmSeq_var arg_2;
  ::CORBA::Boolean result;
};

void _0RL_cd_dc4492deb8062559_b0000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;

}

void _0RL_cd_dc4492deb8062559_b0000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::ULong&)arg_0 <<= _n;

}

void _0RL_cd_dc4492deb8062559_b0000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalBoolean(result);
  arg_1 >>= _n;
  (const AlarmSeq&) arg_2 >>= _n;

}

void _0RL_cd_dc4492deb8062559_b0000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalBoolean();
  (::CORBA::ULong&)arg_1 <<= _n;
  arg_2 = new AlarmSeq;
  (AlarmSeq&)arg_2 <<= _n;

}

const char* const _0RL_cd_dc4492deb8062559_b0000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_c0000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_b0000000* tcd = (_0RL_cd_dc4492deb8062559_b0000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->getActiveAlarm(tcd->arg_0, tcd->arg_1, tcd->arg_2.out());


}

::CORBA::Boolean _objref_GripperExtAxes::getActiveAlarm(::CORBA::ULong maximumNumber, ::CORBA::ULong& numberOfAlarm, ::AlarmSeq_out alarmArray)
{
  _0RL_cd_dc4492deb8062559_b0000000 _call_desc(_0RL_lcfn_dc4492deb8062559_c0000000, "getActiveAlarm", 15);
  _call_desc.arg_0 = maximumNumber;

  _invoke(_call_desc);
  numberOfAlarm = _call_desc.arg_1;
  alarmArray = _call_desc.arg_2._retn();
  return _call_desc.result;


}
// Proxy call descriptor class. Mangled signature:
//  _cboolean_o_cunsigned_plong_o_cLimitSeq
class _0RL_cd_dc4492deb8062559_d0000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_dc4492deb8062559_d0000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::ULong arg_0;
  LimitSeq_var arg_1;
  ::CORBA::Boolean result;
};

void _0RL_cd_dc4492deb8062559_d0000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalBoolean(result);
  arg_0 >>= _n;
  (const LimitSeq&) arg_1 >>= _n;

}

void _0RL_cd_dc4492deb8062559_d0000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalBoolean();
  (::CORBA::ULong&)arg_0 <<= _n;
  arg_1 = new LimitSeq;
  (LimitSeq&)arg_1 <<= _n;

}

const char* const _0RL_cd_dc4492deb8062559_d0000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_e0000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_d0000000* tcd = (_0RL_cd_dc4492deb8062559_d0000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->getSoftLimit(tcd->arg_0, tcd->arg_1.out());


}

::CORBA::Boolean _objref_GripperExtAxes::getSoftLimit(::CORBA::ULong& numberOfData, ::LimitSeq_out limitArray)
{
  _0RL_cd_dc4492deb8062559_d0000000 _call_desc(_0RL_lcfn_dc4492deb8062559_e0000000, "getSoftLimit", 13);


  _invoke(_call_desc);
  numberOfData = _call_desc.arg_0;
  limitArray = _call_desc.arg_1._retn();
  return _call_desc.result;


}
// Proxy call descriptor class. Mangled signature:
//  _cboolean_o_cstring
class _0RL_cd_dc4492deb8062559_f0000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_dc4492deb8062559_f0000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::String_var arg_0;
  ::CORBA::Boolean result;
};

void _0RL_cd_dc4492deb8062559_f0000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalBoolean(result);
  _n.marshalString(arg_0,0);

}

void _0RL_cd_dc4492deb8062559_f0000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalBoolean();
  arg_0 = _n.unmarshalString(0);

}

const char* const _0RL_cd_dc4492deb8062559_f0000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_dc4492deb8062559_01000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_dc4492deb8062559_f0000000* tcd = (_0RL_cd_dc4492deb8062559_f0000000*)cd;
  _impl_GripperExtAxes* impl = (_impl_GripperExtAxes*) svnt->_ptrToInterface(GripperExtAxes::_PD_repoId);
  tcd->result = impl->getVersion(tcd->arg_0.out());


}

::CORBA::Boolean _objref_GripperExtAxes::getVersion(::CORBA::String_out versionMessage)
{
  _0RL_cd_dc4492deb8062559_f0000000 _call_desc(_0RL_lcfn_dc4492deb8062559_01000000, "getVersion", 11);


  _invoke(_call_desc);
  versionMessage = _call_desc.arg_0._retn();
  return _call_desc.result;


}
_pof_GripperExtAxes::~_pof_GripperExtAxes() {}


omniObjRef*
_pof_GripperExtAxes::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::_objref_GripperExtAxes(ior, id);
}


::CORBA::Boolean
_pof_GripperExtAxes::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::GripperExtAxes::_PD_repoId) )
    return 1;
  
  return 0;
}

const _pof_GripperExtAxes _the_pof_GripperExtAxes;

_impl_GripperExtAxes::~_impl_GripperExtAxes() {}


::CORBA::Boolean
_impl_GripperExtAxes::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "servoOn") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_10000000, "servoOn", 8, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "servoOff") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_20000000, "servoOff", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "isPowerOn") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_30000000, "isPowerOn", 10, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "isServoOn") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_40000000, "isServoOn", 10, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "pause") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_50000000, "pause", 6, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "resume") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_60000000, "resume", 7, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "stop") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_70000000, "stop", 5, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "abort") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_80000000, "abort", 6, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "isMoving") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_90000000, "isMoving", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "clearAlarms") ) {

    _0RL_cd_dc4492deb8062559_00000000 _call_desc(_0RL_lcfn_dc4492deb8062559_a0000000, "clearAlarms", 12, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "getActiveAlarm") ) {

    _0RL_cd_dc4492deb8062559_b0000000 _call_desc(_0RL_lcfn_dc4492deb8062559_c0000000, "getActiveAlarm", 15, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "getSoftLimit") ) {

    _0RL_cd_dc4492deb8062559_d0000000 _call_desc(_0RL_lcfn_dc4492deb8062559_e0000000, "getSoftLimit", 13, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "getVersion") ) {

    _0RL_cd_dc4492deb8062559_f0000000 _call_desc(_0RL_lcfn_dc4492deb8062559_01000000, "getVersion", 11, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
_impl_GripperExtAxes::_ptrToInterface(const char* id)
{
  if( id == ::GripperExtAxes::_PD_repoId )
    return (::_impl_GripperExtAxes*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::GripperExtAxes::_PD_repoId) )
    return (::_impl_GripperExtAxes*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
_impl_GripperExtAxes::_mostDerivedRepoId()
{
  return ::GripperExtAxes::_PD_repoId;
}

POA_GripperExtAxes::~POA_GripperExtAxes() {}
