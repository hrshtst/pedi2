#ifndef __PD_FILTER_BW_H__
#define __PD_FILTER_BW_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT bool pdFilterCreateBW(pdFilter *filter, double cf, int dim);

__EXPORT extern pdFilterMethod pd_filter_bw_met;

__END_DECLS

#endif /* __PD_FILTER_BW_H__ */
