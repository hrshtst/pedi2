#ifndef __PD_FILTER_NONE_H__
#define __PD_FILTER_NONE_H__

__BEGIN_DECLS

typedef struct{
  pdFilter base;
} pdFilterNone;

extern void* pdFilterNoneVFTable[];
/* c'tor and d'tor */
__EXPORT void pdFilterNoneInit(pdFilterNone *f, double dt);
__EXPORT void pdFilterNoneDestroy(pdFilterNone *f);

__EXPORT pdFilterNone *pdFilterNoneAlloc();

/* implementations of virtual functions */
__EXPORT void pdFilterNoneUpdate_Imp(pdFilterNone *f);

__END_DECLS

#endif /* __PD_FILTER_NONE_H__ */
