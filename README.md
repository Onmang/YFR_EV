# YFR_EV

//各コードの役割：

//CAN_receive : 受信CAN通信データを表示するのみ、解析しない
//CAN_receive_id : 対象=CPU_5, インバータからのCAN信号を表示する、解析あり、表示する内容の変更はコードを編集して行う
//CAN_send :  適当なCANデータを送信する
//CAN_send_2id : 適当なCANデータを2idに分かれて送信する
//Main_code_Cpu1 : 対象=CPU_1
//Sub_code_Cpu2 : 対象=CPU_2, precharge制御
//VariableResistor_test : APPSセンサーのテストコード
