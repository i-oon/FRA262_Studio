/*
 * feedforward.c
 *
 *  Created on: May 19, 2025
 *      Author: gunda
 */
#include "main.h"
#include "feedforward.h"

float Rev_dis(float rev_en,float pris_en,float direction){
//	float relative;
	float l2;
	float output;
	l2 = 0.04 * (rev_en + pris_en) / 6.28;

	output = ((9.81*(sinf(rev_en)))*((0.1175*0.160)+(0.375*l2))) * (1.3080/(0.01216));



//	return  (9.81*sinf(rev_en))*((0.1175*0.160)+(0.375*l2)) * (1.3080/(0.00481936));
	 return output/145*12;


	//      (         gsin     * (     m1l1    +     m2l2     )) *       ( R/kt)


}

float Pris_dis(float rev_en,float pris_en, float rev_omega){
    float l2 = 0.04 * (rev_en + pris_en) / 6.28;
    float output = (0.375*(9.81*cosf(rev_en)*0.00635 - (rev_omega*rev_omega)*l2)) * (1.3080/(0.01216));
    return output;
    //      (         gsin     * (     m1l1    +     m2l2     )) *       ( R/kt)
}
