// *******************************************************************
//Website:  http://wch.cn
//Email:    tech@wch.cn
//Author:   W.ch 2007.6
// *******************************************************************

// 硬件相关定义, 请根据实际硬件修改本文件
#ifndef	CH423IF_H
#define	CH423IF_H

// CH423接口定义
#define     CH423_I2C_ADDR1     0x40         // CH423的地址
#define     CH423_I2C_MASK      0x3E         // CH423的高字节命令掩码

/*  设置系统参数命令 */ 

#define CH423_SYS_CMD     0x4800     // 设置系统参数命令，默认方式
#define BIT_X_INT         0x08       // 使能输入电平变化中断，为0禁止输入电平变化中断；为1并且DEC_H为0允许输出电平变化中断
#define BIT_DEC_H         0x04       // 控制开漏输出引脚高8位的片选译码
#define BIT_DEC_L         0x02       // 控制开漏输出引脚低8位的片选译码
#define BIT_IO_OE         0x01       // 控制双向输入输出引脚的三态输出，为1允许输出

/*  设置低8位开漏输出命令 */

#define CH423_OC_L_CMD    0x4400     // 设置低8位开漏输出命令，默认方式
#define BIT_OC0_L_DAT     0x01       // OC0为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC1_L_DAT     0x02       // OC1为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC2_L_DAT     0x04       // OC2为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC3_L_DAT     0x08       // OC3为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC4_L_DAT     0x10       // OC4为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC5_L_DAT     0x20       // OC5为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC6_L_DAT     0x40       // OC6为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC7_L_DAT     0x80       // OC7为0则使引脚输出低电平，为1则引脚不输出

/*  设置高8位开漏输出命令 */

#define CH423_OC_H_CMD    0x4600      // 设置低8位开漏输出命令，默认方式
#define BIT_OC8_L_DAT     0x01        // OC8为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC9_L_DAT     0x02        // OC9为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC10_L_DAT    0x04        // OC10为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC11_L_DAT    0x08        // OC11为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC12_L_DAT    0x10        // OC12为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC13_L_DAT    0x20        // OC13为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC14_L_DAT    0x40        // OC14为0则使引脚输出低电平，为1则引脚不输出
#define BIT_OC15_L_DAT    0x80        // OC15为0则使引脚输出低电平，为1则引脚不输出

/* 设置双向输入输出命令 */

#define CH423_SET_IO_CMD   0x6000    // 设置双向输入输出命令，默认方式
#define BIT_IO0_DAT        0x01      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO0为0输出低电平，为1输出高电平
#define BIT_IO1_DAT        0x02      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO1为0输出低电平，为1输出高电平
#define BIT_IO2_DAT        0x04      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO2为0输出低电平，为1输出高电平
#define BIT_IO3_DAT        0x08      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO3为0输出低电平，为1输出高电平
#define BIT_IO4_DAT        0x10      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO4为0输出低电平，为1输出高电平
#define BIT_IO5_DAT        0x20      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO5为0输出低电平，为1输出高电平
#define BIT_IO6_DAT        0x40      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO6为0输出低电平，为1输出高电平
#define BIT_IO7_DAT        0x80      // 写入双向输入输出引脚的输出寄存器，当IO_OE=1,IO7为0输出低电平，为1输出高电平

/* 读取双向输入输出命令 */

#define CH423_RD_IO_CMD		0x4D	// 输入I/O引脚当前状态

#endif
