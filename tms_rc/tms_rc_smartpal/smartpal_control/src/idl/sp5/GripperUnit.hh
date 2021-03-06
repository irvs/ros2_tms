// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __GripperUnit_hh__
#define __GripperUnit_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_GripperUnit
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_GripperUnit
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_GripperUnit
#endif



#ifndef __Common_hh_EXTERNAL_GUARD__
#define __Common_hh_EXTERNAL_GUARD__
#include "Common.hh"
#endif
#ifndef __GripperExtAxes_hh_EXTERNAL_GUARD__
#define __GripperExtAxes_hh_EXTERNAL_GUARD__
#include "GripperExtAxes.hh"
#endif



#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





#ifndef __GripperUnit__
#define __GripperUnit__

class GripperUnit;
class _objref_GripperUnit;
class _impl_GripperUnit;

typedef _objref_GripperUnit* GripperUnit_ptr;
typedef GripperUnit_ptr GripperUnitRef;

class GripperUnit_Helper {
public:
  typedef GripperUnit_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_GripperUnit, GripperUnit_Helper> GripperUnit_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_GripperUnit,GripperUnit_Helper > GripperUnit_out;

#endif

// interface GripperUnit
class GripperUnit {
public:
  // Declarations for this interface type.
  typedef GripperUnit_ptr _ptr_type;
  typedef GripperUnit_var _var_type;

  static _ptr_type _duplicate(_ptr_type);
  static _ptr_type _narrow(::CORBA::Object_ptr);
  static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
  
  static _ptr_type _nil();

  static inline void _marshalObjRef(_ptr_type, cdrStream&);

  static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
    omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
    if (o)
      return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
    else
      return _nil();
  }

  static _core_attr const char* _PD_repoId;

  // Other IDL defined within this scope.
  
};

class _objref_GripperUnit :
  public virtual _objref_GripperExtAxes
{
public:
  ::CORBA::Boolean move(::CORBA::Double pos, ::CORBA::Double velocity, ::CORBA::Double accel);
  ::CORBA::Boolean getFeedback(::CORBA::Double& fbPos);
  ::CORBA::Boolean getCommand(::CORBA::Double& cmdPos);

  inline _objref_GripperUnit()  { _PR_setobj(0); }  // nil
  _objref_GripperUnit(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_GripperUnit();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_GripperUnit(const _objref_GripperUnit&);
  _objref_GripperUnit& operator = (const _objref_GripperUnit&);
  // not implemented

  friend class GripperUnit;
};

class _pof_GripperUnit : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_GripperUnit() : _OMNI_NS(proxyObjectFactory)(GripperUnit::_PD_repoId) {}
  virtual ~_pof_GripperUnit();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_GripperUnit :
  public virtual _impl_GripperExtAxes
{
public:
  virtual ~_impl_GripperUnit();

  virtual ::CORBA::Boolean move(::CORBA::Double pos, ::CORBA::Double velocity, ::CORBA::Double accel) = 0;
  virtual ::CORBA::Boolean getFeedback(::CORBA::Double& fbPos) = 0;
  virtual ::CORBA::Boolean getCommand(::CORBA::Double& cmdPos) = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};




class POA_GripperUnit :
  public virtual _impl_GripperUnit,
  public virtual POA_GripperExtAxes
{
public:
  virtual ~POA_GripperUnit();

  inline ::GripperUnit_ptr _this() {
    return (::GripperUnit_ptr) _do_this(::GripperUnit::_PD_repoId);
  }
};







#undef _core_attr
#undef _dyn_attr



inline void
GripperUnit::_marshalObjRef(::GripperUnit_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_GripperUnit
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_GripperUnit
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_GripperUnit
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_GripperUnit
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_GripperUnit
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_GripperUnit
#endif

#endif  // __GripperUnit_hh__

