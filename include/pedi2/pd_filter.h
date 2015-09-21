#ifndef __PD_FILTER_H__
#define __PD_FILTER_H__

#include <dzco/dz_sys.h>

__BEGIN_DECLS

struct _pdFilter;

typedef struct{
  const char *type;
  void (*destroy)(struct _pdFilter*);
  void (*refresh)(struct _pdFilter*);
  double (*update)(struct _pdFilter*, double dt);
} pdFilterMethod;

typedef struct _pdFilter{
  Z_NAMED_CLASS;
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

__EXPORT void pdFilterDestroyDefault(pdFilter *filter);
__EXPORT void pdFilterRefreshDefault(pdFilter *filter);

__END_DECLS

#include <pedi2/pd_filter_none.h>
#include <pedi2/pd_filter_bw.h>

#endif /* __PD_FILTER_H__ */
