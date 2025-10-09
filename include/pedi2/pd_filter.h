#ifndef __PD_FILTER_H__
#define __PD_FILTER_H__

#include <dzco/dz_sys.h>

__BEGIN_DECLS

struct _pdFilter;

typedef struct{
  const char *typestr;
  void (* _destroy)(struct _pdFilter*);
  void (* _refresh)(struct _pdFilter*);
  double (* _update)(struct _pdFilter*, double);
  struct _pdFilter *(* _clone)(struct _pdFilter*, struct _pdFilter*);
  struct _pdFilter *(* _fromZTK)(struct _pdFilter*, ZTK*);
  void (* _fprintZTK)(FILE*, struct _pdFilter*);
} pdFilterCom;

typedef struct _pdFilter{
  Z_NAMED_CLASS;
  double input;
  double output;
  void *prp;
  pdFilterCom *com;
} pdFilter;

#define pdFilterInput(f)  ( (f)->input )
#define pdFilterOutput(f) ( (f)->output )

#define pdFilterInit(f) do{\
  zNameSet( f, NULL );\
  pdFilterInput(f) = 0.0;\
  pdFilterOutput(f) = 0.0;\
  (f)->prp = NULL;\
  (f)->com = NULL;\
} while(0)

#define pdFilterDestroy(f)  (f)->com->_destroy( f )
#define pdFilterRefresh(f)  (f)->com->_refresh( f )
#define pdFilterUpdate(f,h) (f)->com->_update( f, h )
#define pdFilterClone(o,c)  (o)->com->_clone( o, c )

__EXPORT void pdFilterDefaultDestroy(pdFilter *filter);
__EXPORT void pdFilterDefaultRefresh(pdFilter *filter);

#define ZTK_TAG_PEDI2_FILTER            "pedi2::filter"

#define ZTK_KEY_PEDI2_FILTER_NAME       "name"
#define ZTK_KEY_PEDI2_FILTER_TYPE       "type"
#define ZTK_KEY_PEDI2_FILTER_CUTOFFFREQ "cf"
#define ZTK_KEY_PEDI2_FILTER_DIM        "dim"

__EXPORT pdFilter *pdFilterFromZTK(pdFilter *filter, ZTK *ztk);
__EXPORT void pdFilterFPrintZTK(FILE *fp, pdFilter *filter);

zArrayClass( pdFilterArray, pdFilter );

#define pdFilterArrayInput(arr,i)  pdFilterInput( zArrayElem(arr,i) )
#define pdFilterArrayOutput(arr,i) pdFilterOutput( zArrayElem(arr,i) )

__EXPORT pdFilterArray *pdFilterArrayAlloc(pdFilterArray *arr, int size);
__EXPORT void pdFilterArrayDestroy(pdFilterArray *arr);

__EXPORT pdFilter *pdFilterArrayNameFind(pdFilterArray *arr, const char *name);

__EXPORT void pdFilterArrayUpdate(pdFilterArray *arr, double dt);

__EXPORT pdFilterArray *pdFilterArrayFromZTK(pdFilterArray *arr, ZTK *ztk);
__EXPORT void pdFilterArrayFPrintZTK(FILE *fp, pdFilterArray *arr);

__EXPORT pdFilterArray *pdFilterArrayReadZTK(pdFilterArray *arr, char filename[]);
__EXPORT bool pdFilterArrayWriteZTK(pdFilterArray *arr, char filename[]);

__END_DECLS

#include <pedi2/pd_filter_none.h>
#include <pedi2/pd_filter_bw.h>

__BEGIN_DECLS

/* add a handle to the following list when you create a new filter class. */
#define PD_FILTER_COM_ARRAY \
pdFilterCom *pd_filter_com[] = { \
  &pd_filter_none_com, \
  &pd_filter_bw_com, \
  NULL, \
}

__END_DECLS

#endif /* __PD_FILTER_H__ */
