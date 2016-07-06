#ifndef __PD_MODE_H__
#define __PD_MODE_H__

#include <cure/cure_defs.h>

__BEGIN_DECLS

typedef struct{
  bool standing;
  bool trywalk;
  bool stepping;
  bool walking;
  bool sideways;
  bool following;
  bool braking;
  bool rotating;
} pdMode;

__EXPORT void pdModeInit(pdMode *mode);
__EXPORT void pdModeDestroy(pdMode *mode);

__EXPORT void pdModeUpdate(pdMode *mode);

__EXPORT void pdModeFWrite(FILE *fp, pdMode *mode);
#define pdModeWrite(m) pdModeFWrite( stdout, m )

__END_DECLS

#endif /* __PD_MODE_H__ */
