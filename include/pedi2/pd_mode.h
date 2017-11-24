#ifndef __PD_MODE_H__
#define __PD_MODE_H__

#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_exportdecl.h>

__BEGIN_DECLS

typedef struct{
  bool standing;
  bool trymove;
  bool stepping;
  bool walking;
  bool sideways;
  bool following;
  bool braking;
  bool rotating;
  bool warping;
} pdMode;

__EXPORT void pdModeInit(pdMode *mode);
__EXPORT void pdModeDestroy(pdMode *mode);

__EXPORT void pdModeUpdate(pdMode *mode, pdCmd *cmd, pdState *state);

__EXPORT void pdModeFWrite(FILE *fp, pdMode *mode);
#define pdModeWrite(m) pdModeFWrite( stdout, m )

__END_DECLS

#endif /* __PD_MODE_H__ */
