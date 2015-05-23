#ifndef __PDCZHRZUWCALCREGZMP_FAKE_H__
#define __PDCZHRZUWCALCREGZMP_FAKE_H__

#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

/*** code templete ***/
/* static double __expected_value_FUNCNAME; */
/* #define FUNCNAME(_t1,_t2,_t3) FUNCNAME_fake(_t1,_t2,_t3) */
/* double FUNCNAME_fake(type1 t1, type2 t2, type3 t3) */
/* { */
/*   return __expected_value_FUNCNAME; */
/* } */

#define MOCK_EXPECT_RETURN(func,v) ( __expected_value_##func = (v) )

static double __expected_value_pdCZHrzUCalcRegZMP;
#define pdCZHrzUCalcRegZMP(u,d,v) pdCZHrzUCalcRegZMP_fake( u, d, v )
double pdCZHrzUCalcRegZMP_fake(pdCZHrzU *u, zVec2D delta, zVec2D vel)
{
  (void)u;
  (void)delta;
  (void)vel;
  return __expected_value_pdCZHrzUCalcRegZMP;
}

static double __expected_value_pdCZHrzWCalcRegZMP;
#define pdCZHrzWCalcRegZMP(u,d,v) pdCZHrzWCalcRegZMP_fake( u, d, v )
double pdCZHrzWCalcRegZMP_fake(pdCZHrzW *w, zVec2D delta, zVec2D vel)
{
  (void)w;
  (void)delta;
  (void)vel;
  return __expected_value_pdCZHrzWCalcRegZMP;
}

__END_DECLS

#endif /* __PDCZHRZUWCALCREGZMP_FAKE_H__ */
