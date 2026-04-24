#include "app_maze.h"

#include <string.h>

typedef struct
{
    uint8_t x;
    uint8_t y;
    HeadingTypeDef heading;
} AppMazePosition_t;

static const uint8_t wall_lut[4][3] = {
    {WALL_NORTH, WALL_EAST, WALL_WEST},
    {WALL_EAST, WALL_SOUTH, WALL_NORTH},
    {WALL_SOUTH, WALL_WEST, WALL_EAST},
    {WALL_WEST, WALL_NORTH, WALL_SOUTH}};

static uint8_t maze_map[MAZE_WIDTH][MAZE_HEIGHT];
static AppMazePosition_t current_pos;

void App_Maze_ResetRobotPosition(void)
{
    current_pos.x = 7;
    current_pos.y = 7;
    current_pos.heading = HEADING_NORTH;
}

void App_Maze_ResetState(void)
{
    memset(maze_map, 0, sizeof(maze_map));
    App_Maze_ResetRobotPosition();
}

void App_Maze_AdvanceRobotPosition(void)
{
    switch (current_pos.heading)
    {
    case HEADING_NORTH:
        if (current_pos.y < (MAZE_HEIGHT - 1U))
        {
            current_pos.y++;
        }
        break;
    case HEADING_EAST:
        if (current_pos.x < (MAZE_WIDTH - 1U))
        {
            current_pos.x++;
        }
        break;
    case HEADING_SOUTH:
        if (current_pos.y > 0U)
        {
            current_pos.y--;
        }
        break;
    case HEADING_WEST:
        if (current_pos.x > 0U)
        {
            current_pos.x--;
        }
        break;
    }
}

void App_Maze_UpdateRobotHeading(TurnTypeDef turn_direction)
{
    current_pos.heading = (HeadingTypeDef)((current_pos.heading + turn_direction + 4) % 4);
}

void App_Maze_MapCurrentCell(bool front_wall_detected, bool right_wall_detected, bool left_wall_detected)
{
    uint8_t cell_data = CELL_VISITED;

    if (front_wall_detected)
    {
        cell_data |= wall_lut[current_pos.heading][0];
    }
    if (right_wall_detected)
    {
        cell_data |= wall_lut[current_pos.heading][1];
    }
    if (left_wall_detected)
    {
        cell_data |= wall_lut[current_pos.heading][2];
    }

    maze_map[current_pos.x][current_pos.y] |= cell_data;

    if ((cell_data & WALL_NORTH) && (current_pos.y < (MAZE_HEIGHT - 1U)))
    {
        maze_map[current_pos.x][current_pos.y + 1U] |= WALL_SOUTH;
    }
    if ((cell_data & WALL_SOUTH) && (current_pos.y > 0U))
    {
        maze_map[current_pos.x][current_pos.y - 1U] |= WALL_NORTH;
    }
    if ((cell_data & WALL_EAST) && (current_pos.x < (MAZE_WIDTH - 1U)))
    {
        maze_map[current_pos.x + 1U][current_pos.y] |= WALL_WEST;
    }
    if ((cell_data & WALL_WEST) && (current_pos.x > 0U))
    {
        maze_map[current_pos.x - 1U][current_pos.y] |= WALL_EAST;
    }
}

void App_Maze_SendCurrentCellUpdate(_sUNERBUSHandle *bus)
{
    uint8_t cell_payload[UNERBUS_MAZE_CELL_UPDATE_SIZE] = {
        current_pos.x,
        current_pos.y,
        maze_map[current_pos.x][current_pos.y],
        (uint8_t)current_pos.heading};

    UNERBUS_Write(bus, cell_payload, UNERBUS_MAZE_CELL_UPDATE_SIZE);
    UNERBUS_Send(bus, CMD_UPDATE_MAZE_CELL, (uint8_t)(UNERBUS_CMD_ID_SIZE + UNERBUS_MAZE_CELL_UPDATE_SIZE));
}

uint8_t App_Maze_WriteColumnSyncPayload(uint8_t requested_col, uint8_t *buffer)
{
    if (requested_col >= MAZE_WIDTH)
    {
        return 0;
    }

    uint8_t payload_len = 0;
    buffer[payload_len++] = requested_col;

    for (uint8_t i = 0; i < MAZE_HEIGHT; i++)
    {
        buffer[payload_len++] = maze_map[requested_col][i];
    }

    buffer[payload_len++] = current_pos.x;
    buffer[payload_len++] = current_pos.y;
    buffer[payload_len++] = (uint8_t)current_pos.heading;

    return payload_len;
}
