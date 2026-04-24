#ifndef INC_APP_MAZE_H_
#define INC_APP_MAZE_H_

#include <stdbool.h>
#include <stdint.h>

#include "UNERBUS.h"
#include "app_config.h"

#define APP_MAZE_COLUMN_SYNC_PAYLOAD_SIZE (MAZE_HEIGHT + 4U)

void App_Maze_ResetRobotPosition(void);
void App_Maze_ResetState(void);
void App_Maze_AdvanceRobotPosition(void);
void App_Maze_UpdateRobotHeading(TurnTypeDef turn_direction);
void App_Maze_MapCurrentCell(bool front_wall_detected, bool right_wall_detected, bool left_wall_detected);
void App_Maze_SendCurrentCellUpdate(_sUNERBUSHandle *bus);
uint8_t App_Maze_WriteColumnSyncPayload(uint8_t requested_col, uint8_t *buffer);

#endif /* INC_APP_MAZE_H_ */
