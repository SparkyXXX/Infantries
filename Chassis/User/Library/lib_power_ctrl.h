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

#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
	
typedef struct {
    float mp;
    float kp[2];
    float cp;
    float lp;
    float mn[2];
    float kn[2];
    float cn;
    float ln;
}PCArgsType;

typedef struct{
    float lf;
    float rf;
    float lb;
    float rb;
}OmiWheelCurrentType;

typedef struct{
    float lf;
    float rf;
    float lb;
    float rb;
}OmiWheelSpeedType;

float PredictPower(const PCArgsType pp_args, const float I, const float w);

float PowerControlGetCurrentP(const PCArgsType Arg, const float P, const float w);

float PowerControlGetCurrentN(const PCArgsType Arg, const float P, const float w);

void PowerControl(
    const PCArgsType Arg, const float PowerTarget, float CurrentToSend[4], const float SpeedNow[4]
);

#ifdef __cplusplus
};
#endif