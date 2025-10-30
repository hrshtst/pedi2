#ifndef __PD_FILTER_NONE_H__
#define __PD_FILTER_NONE_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__PEDI2_EXPORT pdFilter *pdFilterNoneCreate(pdFilter *filter);

extern pdFilterCom pd_filter_none_com;

__END_DECLS

#endif /* __PD_FILTER_NONE_H__ */
