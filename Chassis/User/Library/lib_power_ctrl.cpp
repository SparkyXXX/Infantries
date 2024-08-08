/*
*                        ::
*                       :;J7, :,                        ::;7:
*                       ,ivYi, ,                       ;LLLFS:
*                       :iv7Yi                       :7ri;j5PL
*                      ,:ivYLvr                    ,ivrrirrY2X,
*                      :;r@Wwz.7r:                :ivu@kexianli.
*                     :iL7::,:::iiirii:ii;::::,,irvF7rvvLujL7ur
*                    ri::,:,::i:iiiiiii:i:irrv177JX7rYXqZEkvv17
*                 ;i:, , ::::iirrririi:i:::iiir2XXvii;L8OGJr71i
*               :,, ,,:   ,::ir@mingyi.irii:i:::j1jri7ZBOS7ivv,
*                  ,::,    ::rv77iiiriii:iii:i::,rvLq@huhao.Li
*              ,,      ,, ,:ir7ir::,:::i;ir:::i:i::rSGGYri712:
*            :::  ,v7r:: ::rrv77:, ,, ,:i7rrii:::::, ir7ri7Lri
*           ,     2OBBOi,iiir;r::        ,irriiii::,, ,iv7Luur:
*         ,,     i78MBBi,:,:::,:,  :7FSL: ,iriii:::i::,,:rLqXv::
*         :      iuMMP: :,:::,:ii;2GY7OBB0viiii:i:iii:i:::iJqL;::
*        ,     ::::i   ,,,,, ::LuBBu BBBBBErii:i:i:i:i:i:i:r77ii
*       ,       :       , ,,:::rruBZ1MBBqi, :,,,:::,::::::iiriri:
*      ,               ,,,,::::i:  @arqiao.       ,:,, ,:::ii;i7:
*     :,       rjujLYLi   ,,:::::,:::::::::,,   ,:i,:,,,,,::i:iii
*     ::      BBBBBBBBB0,    ,,::: , ,:::::: ,      ,,,, ,,:::::::
*     i,  ,  ,8BMMBBBBBBi     ,,:,,     ,,, , ,   , , , :,::ii::i::
*     :      iZMOMOMBBM2::::::::::,,,,     ,,,,,,:,,,::::i:irr:i:::,
*     i   ,,:;u0MBMOG1L:::i::::::  ,,,::,   ,,, ::::::i:i:iirii:i:i:
*     :    ,iuUuuXUkFu7i:iii:i:::, :,:,: ::::::::i:i:::::iirr7iiri::
*     :     :rk@Yizero.i:::::, ,:ii:::::::i:::::i::,::::iirrriiiri::,
*      :      5BMBBBBBBSr:,::rv2kuii:::iii::,:i:,, , ,,:,:i@petermu.,
*           , :r50EZ8MBBBBGOBBBZP7::::i::,:::::,: :,:,::i;rrririiii::
*               :jujYY7LS0ujJL7r::,::i::,::::::::::::::iirirrrrrrr:ii:
*            ,:  :@kevensun.:,:,,,::::i:i:::::,,::::::iir;ii;7v77;ii;i,
*            ,,,     ,,:,::::::i:iiiii:i::::,, ::::iiiir@xingjief.r;7:i,
*         , , ,,,:,,::::::::iiiiiiiiii:,:,:::::::::iiir;ri7vL77rrirri::
*          :,, , ::::::::i:::i:::i:i::,,,,,:,::i:i:::iir;@Secbone.ii:::
*
* @Date: 2024-04-23 19:47:50
* @LastEditors: KraHsu && 1191393280@qq.com
* @LastEditTime: 2024-04-23 19:47:50
* Copyright (c) 2024 by KraHsu, All Rights Reserved.
*/

#include "lib_power_ctrl.h"
#include "lib_ringbuff.h"
#include "math.h"

float PredictPower(const PCArgsType pp_args, const float I, const float w) {
    const uint8_t SameSign = (I * w >= 0);
    const uint8_t Boundary = (w < 0.3 && w > -0.3);

    if (Boundary) {
        return 0.5
               * (
                   pp_args.mp * I * w + pp_args.kp[0] * I * I + pp_args.kp[1] * w * w + pp_args.lp * fabs(w)
                   + pp_args.mn[0] * I * I * w * w + pp_args.mn[1] * I * w + pp_args.kn[0] * I * I
                   + pp_args.kn[1] * w * w + pp_args.ln * fabs(w)
               );
    }

    if (SameSign) {
        return (pp_args.mp * I * w + pp_args.kp[0] * I * I + pp_args.kp[1] * w * w + pp_args.lp * fabs(w));
    } else {
        return (
            pp_args.mn[0] * I * I * w * w + pp_args.mn[1] * I * w + pp_args.kn[0] * I * I + pp_args.kn[1] * w * w
            + pp_args.ln * fabs(w)
        );
    }
}

float PowerControlGetCurrentP(const PCArgsType Arg, const float P, const float w) {
    float Delta  = sqrt(4 * (P * Arg.kp[0] - Arg.kp[0] * Arg.kp[1] * w * w - Arg.lp * Arg.kp[0] * fabs(w)) + Arg.mp * Arg.mp * w * w);
    if (w < 0) {
        return ((-Arg.mp * w - Delta) / (2 * Arg.kp[0]));
    } else {
        return ((-Arg.mp * w + Delta) / (2 * Arg.kp[0]));
    }
}

float PowerControlGetCurrentN(const PCArgsType Arg, const float P, const float w) {
    float        Delta = sqrt(
        4
            * (P * Arg.kn[0] + Arg.mn[0] * w * w - Arg.kn[0] * Arg.kn[1] * w * w - Arg.kn[0] * Arg.ln * fabs(w)
               - Arg.kn[1] * Arg.mn[0] * w * w * w * w - Arg.ln * Arg.mn[0] * w * w * fabs(w))
        + Arg.mn[1] * Arg.mn[1] * w * w
    );
    if (w < 0) {
        return ((-Arg.mn[1] * w + Delta) / 2 / (Arg.kn[0] + Arg.mn[0] * w * w));
    } else {
        return ((-Arg.mn[1] * w - Delta) / 2 / (Arg.kn[0] + Arg.mn[0] * w * w));
    }
}


void PowerControl(
    const PCArgsType Arg, const float PowerTarget, float CurrentToSend[4], const float SpeedNow[4]
) {
		static sheriff::SlidingWindow<float, 16> MotorSpeedWindow[4];
		static float PCTestW[4] = {0};
		static float ChassisW[4] = {0};
		static float power[4];
	
		for (int i = 0; i < 4; i++)
    {			
        PCTestW[i] -= MotorSpeedWindow[i][0];
        MotorSpeedWindow[i] << SpeedNow[i];
        PCTestW[i] += MotorSpeedWindow[i][15];
			
				ChassisW[i] = PCTestW[i] / 16.0f;
    }
	
    for (int i = 0; i < 4; i++) {
        power[i] = PredictPower(Arg, CurrentToSend[i], SpeedNow[i]);
    }

    if (power[0] + power[1] + power[2] + power[3] <= PowerTarget) {
        return;
    }

    static float TotalPowerPositive, TotalPowerNegative;
    TotalPowerPositive = 0;
    TotalPowerNegative = 0;

    for (uint8_t i = 0; i < 4; i ++) {
        if (power[i] > 0) {
            TotalPowerPositive += power[i];
        } else {
            TotalPowerNegative += power[i];
        }
    }

    float k = (PowerTarget - TotalPowerNegative) / TotalPowerPositive;

    for (int i = 0; i < 4; i++) {
					if (CurrentToSend[i] * SpeedNow[i] >= 0) {
							CurrentToSend[i] = PowerControlGetCurrentP(Arg, power[i] * k, SpeedNow[i]);
					}
    }
}