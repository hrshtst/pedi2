#ifndef __PD_MODE_H__
#define __PD_MODE_H__

#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>

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
  bool balancing; /* holding an intentional single-support stance */
} pdMode;

__PEDI2_EXPORT void pdModeInit(pdMode *mode);
__PEDI2_EXPORT void pdModeDestroy(pdMode *mode);

__PEDI2_EXPORT void pdModeUpdate(pdMode *mode, pdCmd *cmd, pdState *state);

__PEDI2_EXPORT void pdModeFWrite(FILE *fp, pdMode *mode);
#define pdModeWrite(m) pdModeFWrite( stdout, m )

__END_DECLS

#endif /* __PD_MODE_H__ */
