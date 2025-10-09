#ifndef __PD_FILTER_BW_H__
#define __PD_FILTER_BW_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT pdFilter *pdFilterBWCreate(pdFilter *filter, double cf, uint dim);

extern pdFilterCom pd_filter_bw_com;

__END_DECLS

#endif /* __PD_FILTER_BW_H__ */
