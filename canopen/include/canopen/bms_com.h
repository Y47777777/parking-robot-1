#ifndef __FINDRAIL_H__
#define __FINDRAIL_H__

//保护参数
typedef struct{
    int Addr;   //保护板RS485地址1~255
    int CellNum;    // 电池节数5~16
    int CSGLimitEn;     //限流使能
    float EngDesign;  //设计容量,(0.1AH)
    float Rsense;   //采样电阻大小0.01mR)
    int Vref;   //参考电压 mv

    unsigned char B_Mode;   //均衡模式0~2,0:不均衡1:充电均衡2充电+静态均衡
    unsigned char B_THDIS;  //均衡高温禁止值 40 表示0°C 65 表示25°C
    unsigned char B_TLDIS;  //均衡低温禁止值
    unsigned short B_VStart;    //均衡启动电压(mV)
    unsigned short B_Vdiff;     //均衡启动压差(mV)

    unsigned short W_Vcell_H;   //单节高压警告值mv
    unsigned short W_VCell_L;   //单节低压警告值
    unsigned short W_VBAT_H;   //电池高压警告值
    unsigned short W_VBAT_L;   //电池低压警告值

    unsigned char W_Tcell_H;   //电芯高温警告值
    unsigned char W_Tcell_L;   //电芯低温警告值
    unsigned char W_Tenv_H;   //环境高温警告值
    unsigned char W_Tenv_L;   //环境低温警告值
    unsigned char W_Tfet_H;   //功率高温警告值
    unsigned char W_Tfet_L;   //功率低温警告值

    unsigned short W_CURR_C;   //充电电流警告值 0.01A
    unsigned short W_CURR_D;   //放电电流警告值
    unsigned short W_VDIFF_H;   //压差报警值
    unsigned short W_VDIFF_L;   //压差报警解除值

    unsigned short OVPVal;   //单体过充电压
    unsigned short OVPDly;   //单体过充保护延时
    unsigned short OVPRel;   //单体过充恢复电压
    unsigned short UVPVal;   //单体过放电压
    unsigned short UVPDly;   //单体过放保护延时
    unsigned short UVPRel;   //单体过放恢复电压

    unsigned short BOVPVal;   //电池总体过充电压
    unsigned short BOVPDly;   //电池总体过充保护延时
    unsigned short BOVPRel;   //电池总体过充恢复电压
    unsigned short BUVPVal;   //电池过放电压
    unsigned short BUVPDly;   //电池过放保护延时
    unsigned short BUVPRel;   //电池过放恢复电压

    unsigned char CTcellHPro;   //电芯充电高温保护
    unsigned char CTcellHRel;   //电芯充电高温保护恢复
    unsigned char CTcellLPro;   //电芯充电低温保护
    unsigned char CTcellLRel;   //电芯充电低温保护恢复

    unsigned char DTcellHPro;   //电芯放电高温保护
    unsigned char DTcellHRel;   //电芯放电高温保护恢复
    unsigned char DTcellLPro;   //电芯放电低温保护
    unsigned char DTcellLRel;   //电芯放电低温保护恢复

    unsigned char TenvHPro;   //电芯环境高温保护
    unsigned char TenvHRel;   //电芯环境高温保护恢复
    unsigned char TenvLPro;   //电芯环境低温保护
    unsigned char TenvLRel;   //电芯环境低温保护恢复
    unsigned char TfetHPro;   //电芯功率高温保护
    unsigned char TfetHRel;   //电芯功率高温保护恢复
    unsigned char TfetLPro;   //电芯功率低温保护
    unsigned char TfetLRel;   //电芯功率低温保护恢复

    unsigned short CC_PRO_VAL;   //充电电流保护值
    unsigned short CC_PRO_PDLY;   //充电电流保护延时
    unsigned short CC_PRO_RDLY;   //充电电流恢复延时
    unsigned short CC_PRO_LOCK;   //充电电流保护锁定

    unsigned short CD1_PRO_VAL;   //一级放电保护值
    unsigned short CD1_PRO_PDLY;   //一级放电电流保护延时
    unsigned short CD1_PRO_RDLY;   //一级放电电流恢复延时
    unsigned short CD1_PRO_LOCK;   //一级放电电流保护锁定

    unsigned short CD2_PRO_VAL;   //二级放电保护值
    unsigned short CD2_PRO_PDLY;   //二级放电电流保护延时
    unsigned short CD2_PRO_RDLY;   //二级放电电流恢复延时
    unsigned short CD2_PRO_LOCK;   //二级放电电流保护锁定

    unsigned char SHORT_VAL;   //短路电压保护值
    unsigned char SHORT_RDLY;   //短路延时值
    unsigned char SHORT_LOCK;   //短路锁定值
    unsigned char HEAT_EN;   //加热功能使能
    unsigned char HEAT_TSTART;   //加热开启温度
    unsigned char HEAT_TEND;   //加热关闭温度
}BMSProtectParaData;



#endif