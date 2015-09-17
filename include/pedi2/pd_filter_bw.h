#ifndef __PD_FILTER_BW_H__
#define __PD_FILTER_BW_H__

__BEGIN_DECLS

typedef struct{
  pdFilter base;

  dzSys _sys;
} pdFilterBW;

extern void* pdFilterBWVFTable[];
/* c'tor and d'tor */
__EXPORT void pdFilterBWInit(pdFilterBW *bwf, double dt, double cf, int dim);
__EXPORT void pdFilterBWDestroy(pdFilterBW *bwf);

/* implementations of virtual functions */
__EXPORT void pdFilterBWUpdate_Imp(pdFilterBW *bwf);

__END_DECLS

#endif /* __PD_FILTER_BW_H__ */
