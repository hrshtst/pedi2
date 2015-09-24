#ifndef __PD_FILTER_NONE_H__
#define __PD_FILTER_NONE_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT bool pdFilterCreateNone(pdFilter *filter);

extern pdFilterMethod pd_filter_none_met;

__END_DECLS

#endif /* __PD_FILTER_NONE_H__ */
