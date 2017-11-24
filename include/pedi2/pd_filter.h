#ifndef __PD_FILTER_H__
#define __PD_FILTER_H__

#include <dzco/dz_sys.h>
#include <pedi2/pd_exportdecl.h>

__BEGIN_DECLS

struct _pdFilter;

typedef struct{
  const char *type;
  void (*destroy)(struct _pdFilter*);
  void (*refresh)(struct _pdFilter*);
  double (*update)(struct _pdFilter*, double dt);
  struct _pdFilter *(*clone)(struct _pdFilter*, struct _pdFilter*);
  struct _pdFilter *(*fread)(FILE *fp, struct _pdFilter*);
} pdFilterMethod;

typedef struct _pdFilter{
  Z_NAMED_CLASS
  double input;
  double output;
  void *_prm;
  pdFilterMethod *_met;
} pdFilter;

#define pdFilterInput(f)  ( (f)->input )
#define pdFilterOutput(f) ( (f)->output )

#define pdFilterInit(f) do{\
  zNameSet( f, NULL );\
  pdFilterInput(f) = 0.0;\
  pdFilterOutput(f) = 0.0;\
  (f)->_prm = NULL;\
  (f)->_met = NULL;\
} while(0)

#define pdFilterDestroy(f)  (f)->_met->destroy( f )
#define pdFilterRefresh(f)  (f)->_met->refresh( f )
#define pdFilterUpdate(f,h) (f)->_met->update( f, h )
#define pdFilterClone(s,d)  (s)->_met->clone( s, d )

__EXPORT void pdFilterDestroyDefault(pdFilter *filter);
__EXPORT void pdFilterRefreshDefault(pdFilter *filter);

#define PD_FILTER_TAG "filter"
__EXPORT pdFilter *pdFilterFRead(FILE *fp, pdFilter *filter);

zArrayClass( pdFilterArray, pdFilter );

#define pdFilterArrayInput(arr,i)  pdFilterInput( zArrayElem(arr,i) )
#define pdFilterArrayOutput(arr,i) pdFilterOutput( zArrayElem(arr,i) )

__EXPORT bool pdFilterArrayAlloc(pdFilterArray *arr, int n);
__EXPORT void pdFilterArrayDestroy(pdFilterArray *arr);

__EXPORT pdFilter *pdFilterArrayNameFind(pdFilterArray *arr, const char *name);

__EXPORT void pdFilterArrayUpdate(pdFilterArray *arr, double dt);

__EXPORT bool pdFilterArrayFRead(FILE *fp, pdFilterArray *arr);

__END_DECLS

#include <pedi2/pd_filter_none.h>
#include <pedi2/pd_filter_bw.h>

#endif /* __PD_FILTER_H__ */
