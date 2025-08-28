#include "wrapper.hpp"
#include "stm32f4xx_nucleo_bus.h"
#include "vl53l4cx.h"
#include "vl53lx_api.h"


// 書き込み用の関数
static int32_t vl53l4cx_io_write(uint16_t DevAddr, uint8_t *pdata, uint16_t count)
{

  return BSP_I2C1_Send(DevAddr, pdata, count);
}

// 読み取り用の関数
static int32_t vl53l4cx_io_read(uint16_t DevAddr, uint8_t *pdata, uint16_t count)
{

  return BSP_I2C1_Recv(DevAddr, pdata, count);
}

// VL53L4CXのオブジェクト
static VL53L4CX_Object_t vl53l4cx;

// VL53L4CX_IO_t のオブジェクト
// 初期化などに使用する関数のポインタを設定
static VL53L4CX_IO_t vl53l4cx_io = {

	// I2Cの初期化関数
	.Init = BSP_I2C1_Init,

	// I2Cの終了関数
	.DeInit = BSP_I2C1_DeInit,

	// I2Cアドレス（ライブでの定数値）
	.Address = VL53L4CX_DEVICE_ADDRESS,

	// センサーへの書き込み関数
	.WriteReg = vl53l4cx_io_write,

	// センサーからの読み取り関数
	.ReadReg = vl53l4cx_io_read,

	// 現在の時間取得関数
	.GetTick = BSP_GetTick
};

// 　設定用の構造体
static VL53L4CX_ProfileConfig_t vl53l4cx_profileConfig_t{

	// 測定モード(SHORT, MEDIUM, LONG)
	VL53L4CX_PROFILE_MEDIUM,

	// 測定時間(ms)
	95U,

	// 読み取りレート(hz)
	10U,

	// Ambientの出力設定(0 or 1)
	1U,

	// Signalの出力設定(0 or 1)
	1U
};

// 測定範囲の設定
// 縦横ともに分割が0~15であらわされているので7,8を中心として、上限と下限を設定する
static VL53L4CX_ROIConfig_t vl53l4cx_roiConfig_t{
	7U, /* TopLeftX  : 0 */
	7U, /* TopLeftY  : 0 */
	8U,/* BotRightX : 15 (max) */
	8U /* BotRightY : 15 (max) */
};


void init(){


	while(VL53L4CX_RegisterBusIO(&vl53l4cx, &vl53l4cx_io) != 0){

		static uint8_t error_count = 0;

		if(error_count++ > 100){

			printf("InitError\n");
			while(1);
		}
	}

	printf("InitOK\n");

	// 登録した関数を使って初期化をする
	VL53L4CX_Init(&vl53l4cx);

	// ProflieConfingの設定(測定レートなど）
	VL53L4CX_ConfigProfile(&vl53l4cx, &vl53l4cx_profileConfig_t);

	// ROIConfigの設定（測定範囲）
	VL53L4CX_ConfigROI(&vl53l4cx, &vl53l4cx_roiConfig_t);

	// 割り込みの設定（特定範囲に入ったらGPIO出力などができる）
	//VL53L4CX_ConfigIT(&vl53l4cx, pITConfig)

    // 測定開始（連続測距モード）
	VL53L4CX_Start(&vl53l4cx, VL53L4CX_MODE_BLOCKING_CONTINUOUS);
}

void loop(){

	// 結果を取得するための構造体のインスタンスを作成
	static VL53L4CX_Result_t res;

	// 測定結果を取得
	VL53L4CX_GetDistance(&vl53l4cx, &res);

	if(res.ZoneResult[0].Status[0] == 1){

		printf("Error: GetDistance\n");
	}
	else{

		float best_SNR = 0.0f;
		uint8_t best_index = 0;

		for(uint8_t i=0; i<res.ZoneResult[0].NumberOfTargets; i++){

			float SNR = res.ZoneResult[0].Signal[i] / res.ZoneResult[0].Ambient[i];

			if(best_SNR < SNR){

				best_index = i;
			}
		}

        printf("%8s %7s %7s\n","Distance", "Ambient", "Signal");
        printf("%8d, %7.3lf, %7.3lf\n", res.ZoneResult[0].Distance[best_index], res.ZoneResult[0].Ambient[best_index], res.ZoneResult[0].Signal[best_index]);
	}

	// 測定終了
	//VL53L4CX_Stop(&vl53l4cx);
}





