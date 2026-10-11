#include "board.h"

Board_Tx_Data_t board_tx_data;

Board_Rx_Data_t board_rx_data;

Board_t board = {
	.rx_data = &board_rx_data,
	.tx_data = &board_tx_data,
};
