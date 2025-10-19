/*
  Physical DFS Maze Explorer (3 sensors; inferred back) + EEPROM save/load
  - -1 = unknown, 0 = open, 1 = wall
  - Sensors: FRONT / LEFT / RIGHT only
  - If F/L/R are walls, infer BACK is open (no fully covered cell) and move 180°
  - Borders treated as walls
  - EEPROM header + CRC16 for reliable persistence

  Serial Monitor: 115200 baud, Newline
  Menu:
    e = Explore now (and save)
    p = Print current maze
    l = Load saved maze from EEPROM
    c = Clear saved maze header
    a = A* shortest path to a goal (after explore)
*/

#include <Arduino.h>
#include <EEPROM.h>  // Teensy emulated EEPROM
#include <ctype.h>   // for tolower()
#include <limits.h>

#include <Wire.h>
#include <VL53L0X.h>
// Create VL53L0X sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

// XSHUT pins for VL53L0X sensors
const int XSHUT1 = 40;
const int XSHUT2 = 41;

#define DIP1 38  // search
// #define DIP2 36   // run

// // New I2C addresses to assign
#define SENSOR1_ADDR 0x30
#define SENSOR2_ADDR 0x31

#define MPU6050_ADDR 0x68

// === Kalman Filter Class ===
class KalmanFilter {
public:
  KalmanFilter(float processNoise, float measurementNoise, float estimatedError, float initialValue) {
    q = processNoise;
    r = measurementNoise;
    p = estimatedError;
    x = initialValue;
  }

  float update(float measurement) {
    p += q;
    float k = p / (p + r);
    x += k * (measurement - x);
    p *= (1 - k);
    return x;
  }

private:
  float q, r, p, x;
};

KalmanFilter kalman1(20, 10.0, 5.0, 100.0);
KalmanFilter kalman2(20, 10.0, 5.0, 100.0);

KalmanFilter kalman3(0.7, 10.0, 10.0, 75);  // q,r,p,x init
KalmanFilter kalman4(0.7, 10.0, 10.0, 75);  // q,r,p,x init

// === ToF Sensor Pins ===
const int frontRightTOF = 17;  // ToF front right
const int frontLeftTOF = 15;   // ToF front left

// Distance Sensor Pins
const uint8_t leftSensorPin = 14;
const uint8_t rightSensorPin = 16;

int targetDistance_mm = 80;

#define SEARCH 0
#define CALIBRATING 1
#define RUN 2

int MODE = SEARCH;

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

int initial_direction = EAST;
int expl_end_dir = NORTH;
int run_end_dir = NORTH;

// === Motor Pins ===
const int leftMotor_pin1 = 2;
const int leftMotor_pin2 = 3;
const int rightMotor_pin1 = 10;
const int rightMotor_pin2 = 4;

// === Encoder Pins ===
const int leftEncoderPinA = 5;
const int leftEncoderPinB = 6;
const int rightEncoderPinA = 28;
const int rightEncoderPinB = 29;


// === Encoder Variables ===
volatile long leftEncoder_count = 0;
volatile long rightEncoder_count = 0;

int left_error = 0;
int right_error = 0;

int total_leftCount = 0;
int total_rightCount = 0;
// === Motor Speed ===
const int baseSpeed = 180;  //120

// === PID Parameters for encoders ===
float Kp = 1.8;
float Ki = 0.99;
float Kd = 0.9;

int correctionStep = 10;
int lastLeftCorrectionCount = 0;
int lastRightCorrectionCount = 0;
int lastError = 0;
float integral = 0;

#define CORRECTION_OFFSET 140

// PID for double wall PID
bool perpendicular_allowed = true;

float Kp_tof = 1.5;
float Ki_tof = 0.001;
float Kd_tof = 5.0;
// PID variables
float error_tof = 0;
float lastError_tof = 0;
float integral_tof = 0;

//PID for right wall
float Kp_right_tof = 1.5;
float Ki_right_tof = 0.00;  //0.001
float Kd_right_tof = 10;    //1
// PID variables
float error_right_tof = 0;
float lastError_right_tof = 0;
float integral_right_tof = 0;

//PID for left wall
float Kp_left_tof = 1.2;
float Ki_left_tof = 0.05;
float Kd_left_tof = 8;  //0.5
// PID variables
float error_left_tof = 0;
float lastError_left_tof = 0;
float integral_left_tof = 0;

//gyro pid
float gyro_integral = 0;
float gyro_lastError = 0;

float gyro_Kp = 4.0;
float gyro_Ki = 0.001;
float gyro_Kd = 1.0;

const float Distance = 192;
const float wheelDiameterMM = 34.0;  // example: 70 mm wheel
const int encoderCPR = 716;          // ticks per wheel revolution
const float wheelCircumferenceMM = PI * wheelDiameterMM;
const float ticksPerMM = encoderCPR / wheelCircumferenceMM;
const float encoder_distance = 192 / (wheelCircumferenceMM)*encoderCPR;
const float distance_between_wheels = 90;

int lastLeftCorrectionLeft = 0;
int lastLeftCorrectionRight = 0;

int leftSpeed = baseSpeed;
int rightSpeed = baseSpeed;

// float per_Kp = 0.4;
// float per_Ki = 0.00;
// float per_Kd = 0.3;
float per_Kp = 0.6;
float per_Ki = 0.01;
float per_Kd = 1.5;

// === Yaw angle from MPU6050 ===
float yaw = 0.0;
unsigned long lastTime = 0;
//PID selection
static bool pidSelected = false;
static enum { DOUBLE_WALL,
              LEFT_WALL,
              RIGHT_WALL,
              NO_WALL } currentPID;

// ===== Constants =====
enum { WALL_TOP = 0,
       WALL_RIGHT = 1,
       WALL_BOTTOM = 2,
       WALL_LEFT = 3 };

#ifndef MAZE_SIZE
#define MAZE_SIZE 16  // adjust as needed
#endif

// ===== Helpers =====
struct Point {
  int y, x;
};

// (A* uses Cell but it's the same shape; keep separate for clarity)
struct Cell {
  int y, x;
};

Cell start_{ 0, 0 };
// Cell goal_{ 3, 2 };

int expl_end_x = 0;
int expl_end_y = 0;

struct Node {
  Cell c;
  int g;          // cost so far
  int f;          // total = g + h
  int parentIdx;  // index into 'closed' array
};

// simple fixed list for open set
struct ListN {
  Node arr[MAZE_SIZE * MAZE_SIZE * 4];
  int n;
};
void DFS_listn_init(ListN &L);

// N, E, S, W
const int DIRS[4][2] = { { -1, 0 }, { 0, 1 }, { 1, 0 }, { 0, -1 } };
const char *DIR_NAMES[4] = { "NORTH", "EAST", "SOUTH", "WEST" };

// ===== Maze state =====
// -1 = unknown, 0 = open, 1 = wall
int8_t mazeArr[MAZE_SIZE][MAZE_SIZE][4];

// ---------------- Movement stubs (replace with real motor control) -------------
void turnright() {
  Serial.println("↻ turn RIGHT 90°");
  turnLeft_PID(-88);
}
void turnleft() {
  Serial.println("↺ turn LEFT 90°");
  turnLeft_PID(88);
}
void turnback() {
  Serial.println("⮌ turn 180°");
  turnLeft_PID(88);
  alignPerpendicularToWall(60);
  turnLeft_PID(88);
}
void forward() {
  Serial.println("⬆ forward 1 cell");
  moveOneCellForward(getEncoderCountFromDistance(19.2));
}

// Turn from currentDir to targetDir using your primitives
void performTurnTo(int currentDir, int targetDir) {
  int diff = (targetDir - currentDir + 4) % 4;
  if (diff == 0) {
    // no turn
  } else if (diff == 1) {
    turnright();
  } else if (diff == 2) {
    turnback();
  } else if (diff == 3) {
    turnleft();
  }
}

bool inBounds(int y, int x) {
  return (y >= 0 && y < MAZE_SIZE && x >= 0 && x < MAZE_SIZE);
}

int directionFromTo(Point a, Point b) {
  int dy = b.y - a.y, dx = b.x - a.x;
  for (int d = 0; d < 4; ++d) {
    if (dy == DIRS[d][0] && dx == DIRS[d][1]) return d;
  }
  return -1;
}

// === Robust Y/N reader that ignores stray CR/LF and only accepts y or n ===
// char readYN(const char *prompt) {
//   Serial.print(prompt);
//   while (true) {
//     while (!Serial.available()) { /* wait */
//     }
//     char c = Serial.read();
//     if (c == '\r' || c == '\n' || c == ' ' || c == '\t') continue;
//     c = (char)tolower((unsigned char)c);
//     if (c == 'y' || c == 'n') {
//       while (Serial.available()) {
//         char t = Serial.read();
//         if (t == '\n') break;
//       }
//       return c;
//     }
//   }
// }

// === Robust integer reader (newline-terminated) ===
int readInt(const char *prompt) {
  // Serial.print(prompt);
  // String buf;
  // while (true) {
  //   while (!Serial.available()) { /* wait */
  //   }
  //   char c = Serial.read();
  //   if (c == '\r') continue;
  //   if (c == '\n') {
  //     if (buf.length() == 0) {
  //       Serial.print(prompt);
  //       continue;
  //     }
  //     return buf.toInt();
  //   }
  //   buf += c;
  // }
  return 0;
}

// ===== Sensor I/O (3 sides only) =====
void readIRSensors(int y, int x, int dir, bool &front, bool &left, bool &right) {
  Serial.println();
  Serial.print("🤖 Robot @ (");
  Serial.print(y);
  Serial.print(",");
  Serial.print(x);
  Serial.print(") facing ");
  Serial.println(DIR_NAMES[dir]);

  // front = (readYN("Wall FRONT? (y/n): ") == 'y');
  // left = (readYN("Wall LEFT?  (y/n): ") == 'y');
  // right = (readYN("Wall RIGHT? (y/n): ") == 'y');
  front = isFrontWallAvailable();
  left = isLeftWallAvailable();
  right = isRightWallAvailable();
}

// Update only FRONT/LEFT/RIGHT (do not touch BACK here)
void updateWalls(int y, int x, int dir, bool front, bool left, bool right) {
  auto setEdge = [&](int d, bool isWall) {
    int ny = y + DIRS[d][0], nx = x + DIRS[d][1];
    if (inBounds(ny, nx)) {
      mazeArr[y][x][d] = isWall ? 1 : 0;
      mazeArr[ny][nx][(d + 2) % 4] = isWall ? 1 : 0;
    } else {
      // outside boundary is a wall
      mazeArr[y][x][d] = 1;
    }
  };

  setEdge(dir, front);            // front
  setEdge((dir + 3) % 4, left);   // left
  setEdge((dir + 1) % 4, right);  // right
}

// Any visited cell with a known-open *or unknown* edge to an unvisited neighbor?
bool hasFrontier(bool visited[MAZE_SIZE][MAZE_SIZE]) {
  for (int y = 0; y < MAZE_SIZE; ++y) {
    for (int x = 0; x < MAZE_SIZE; ++x) {
      if (!visited[y][x]) continue;
      for (int d = 0; d < 4; ++d) {
        int ny = y + DIRS[d][0], nx = x + DIRS[d][1];
        if (!inBounds(ny, nx)) continue;
        // frontier if edge is open OR unknown to an unvisited neighbor
        if ((mazeArr[y][x][d] == 0 || mazeArr[y][x][d] == -1) && !visited[ny][nx]) return true;
      }
    }
  }
  return false;
}

// ===== DFS with inferred 180° move and UNKNOWN probing =====
int explorePhysical(int startY, int startX, int startDir,
                    Point movePathOut[MAZE_SIZE * MAZE_SIZE * 8],
                    bool exploredOut[MAZE_SIZE][MAZE_SIZE]) {
  bool visited[MAZE_SIZE][MAZE_SIZE];
  for (int y = 0; y < MAZE_SIZE; ++y)
    for (int x = 0; x < MAZE_SIZE; ++x) visited[y][x] = false;

  Point breadcrumb[MAZE_SIZE * MAZE_SIZE * 8];
  int breadcrumbTop = 0;

  int y = startY, x = startX, dir = startDir;
  int moveLen = 0;
  movePathOut[moveLen++] = { y, x };

  while (true) {
    bool firstTimeHere = !visited[y][x];
    if (firstTimeHere) {
      visited[y][x] = true;
      bool front = false, left = false, right = false;
      readIRSensors(y, x, dir, front, left, right);
      updateWalls(y, x, dir, front, left, right);
    }

    // 1) Candidates: prefer FRONT, then LEFT, then RIGHT.
    //    Allow OPEN (0) and UNKNOWN (-1). If UNKNOWN, probe before moving.
    struct Cand {
      int ny, nx, ndir, state;
    };  // state: 0=open, -1=unknown
    Cand candidates[3];
    int cc = 0;
    const int pref[3] = { 0, -1, 1 };  // front, left, right
    for (int i = 0; i < 3; ++i) {
      int ndir = (dir + pref[i] + 4) % 4;
      int ny = y + DIRS[ndir][0], nx = x + DIRS[ndir][1];
      if (!inBounds(ny, nx)) continue;
      int edge = mazeArr[y][x][ndir];
      if (!visited[ny][nx] && (edge == 0 || edge == -1)) {
        candidates[cc++] = { ny, nx, ndir, edge };
      }
    }

    if (cc > 0) {
      int ny = candidates[0].ny, nx = candidates[0].nx;
      int ndir = candidates[0].ndir, state = candidates[0].state;

      // Face the candidate direction first
      performTurnTo(dir, ndir);

      // If unknown, PROBE here: read sensors oriented to ndir; update walls.
      if (state == -1) {
        bool f = false, l = false, r = false;
        readIRSensors(y, x, ndir, f, l, r);  // facing ndir now
        updateWalls(y, x, ndir, f, l, r);
      }

      // Move only if now known-open
      if (mazeArr[y][x][ndir] == 0) {
        if (breadcrumbTop < (int)(MAZE_SIZE * MAZE_SIZE * 8)) breadcrumb[breadcrumbTop++] = { y, x };
        forward();
        y = ny;
        x = nx;
        dir = ndir;
        movePathOut[moveLen++] = { y, x };
        continue;
      }
      // otherwise it turned out to be a wall; fall through to other logic
    }

    // 2) Inference rule: NO covered cell -> back must be open and move 180°
    bool fWall = (mazeArr[y][x][dir] == 1);
    bool lWall = (mazeArr[y][x][(dir + 3) % 4] == 1);
    bool rWall = (mazeArr[y][x][(dir + 1) % 4] == 1);

    int backDir = (dir + 2) % 4;
    int by = y + DIRS[backDir][0], bx = x + DIRS[backDir][1];

    if (fWall && lWall && rWall && inBounds(by, bx) && !visited[by][bx]) {
      if (mazeArr[y][x][backDir] != 1) {
        // infer open
        mazeArr[y][x][backDir] = 0;
        mazeArr[by][bx][(backDir + 2) % 4] = 0;
      }
      if (breadcrumbTop < (int)(MAZE_SIZE * MAZE_SIZE * 8)) breadcrumb[breadcrumbTop++] = { y, x };

      // MOVE: turn 180° and go forward one cell
      performTurnTo(dir, backDir);  // call turnback()
      forward();

      // Update logical pose
      y = by;
      x = bx;
      dir = backDir;
      movePathOut[moveLen++] = { y, x };
      continue;
    }

    // 3) No immediate move; if no frontier left, finish
    if (!hasFrontier(visited)) {
      Serial.println("✅ Maze fully explored (no more cells to backtrack to).");
      expl_end_dir = dir;
      expl_end_x = x;
      expl_end_y = y;
      break;
    }

    // 4) Backtrack along breadcrumb to resume elsewhere (standard DFS)
    Serial.print("🔙 Dead-end at (");
    Serial.print(y);
    Serial.print(",");
    Serial.print(x);
    Serial.print(") facing ");
    Serial.println(DIR_NAMES[dir]);

    while (breadcrumbTop > 0) {
      Point back = breadcrumb[--breadcrumbTop];
      int backDir2 = directionFromTo({ y, x }, back);

      if (backDir2 >= 0 && mazeArr[y][x][backDir2] == 1) {
        Serial.print("❌ Unexpected wall while backtracking from (");
        Serial.print(y);
        Serial.print(",");
        Serial.print(x);
        Serial.print(") to (");
        Serial.print(back.y);
        Serial.print(",");
        Serial.print(back.x);
        Serial.println(")");
      }

      Serial.print("⬅ Backtracking: (");
      Serial.print(movePathOut[moveLen - 1].y);
      Serial.print(",");
      Serial.print(movePathOut[moveLen - 1].x);
      Serial.print(") -> (");
      Serial.print(back.y);
      Serial.print(",");
      Serial.print(back.x);
      Serial.println(")");

      // MOVE: turn to the back breadcrumb cell and go forward
      if (backDir2 >= 0) performTurnTo(dir, backDir2);
      forward();

      // Update logical pose
      y = back.y;
      x = back.x;
      if (backDir2 >= 0) dir = backDir2;
      movePathOut[moveLen++] = { y, x };

      // Can we resume here? (allow OPEN or UNKNOWN)
      bool resumed = false;
      int resumeDir = -1;

      for (int d = 0; d < 4; ++d) {
        int ny = y + DIRS[d][0], nx = x + DIRS[d][1];
        if (inBounds(ny, nx) && (mazeArr[y][x][d] == 0 || mazeArr[y][x][d] == -1) && !visited[ny][nx]) {
          resumeDir = d;
          resumed = true;
          break;
        }
      }

      if (resumed) {
        // Orient now so the turn is visible at the final backtrack cell
        if (resumeDir >= 0 && resumeDir != dir) {
          Serial.print("↷ Orienting to resume toward ");
          Serial.println(DIR_NAMES[resumeDir]);
          performTurnTo(dir, resumeDir);
          dir = resumeDir;
        }
        // don't move forward here; next loop iteration will probe/move
        break;
      }
    }
  }

  // outputs
  for (int yy = 0; yy < MAZE_SIZE; ++yy)
    for (int xx = 0; xx < MAZE_SIZE; ++xx)
      exploredOut[yy][xx] = visited[yy][xx];

  // Print full path
  Serial.println();
  Serial.println("📍 Full Movement Path (including backtracking):");
  for (int i = 0; i < moveLen; ++i) {
    if (i) Serial.print(" -> ");
    Serial.print("(");
    Serial.print(movePathOut[i].y);
    Serial.print(",");
    Serial.print(movePathOut[i].x);
    Serial.print(")");
  }
  Serial.println();

  return moveLen;
}

// ====== A* PATHFINDING (uses explored mask) ======
void DFS_listn_init(ListN &L) {
  L.n = 0;
}

void DFS_listn_add(ListN &L, const Node &v) {
  if (L.n < (int)(sizeof(L.arr) / sizeof(L.arr[0]))) L.arr[L.n++] = v;
}

int DFS_listn_findMinF(const ListN &L) {
  int best = -1, bestF = INT_MAX;
  for (int i = 0; i < L.n; ++i) {
    if (L.arr[i].f < bestF) {
      bestF = L.arr[i].f;
      best = i;
    }
  }
  return best;
}

Node DFS_listn_takeAt(ListN &L, int idx) {
  Node v = L.arr[idx];
  // compact by swapping last
  L.arr[idx] = L.arr[L.n - 1];
  L.n--;
  return v;
}

// A* helpers
bool DFS_in_bounds(int y, int x) {
  return inBounds(y, x);
}
bool DFS_sameCell(Cell a, Cell b) {
  return a.y == b.y && a.x == b.x;
}
int DFS_manhattan(Cell a, Cell b) {
  return abs(a.y - b.y) + abs(a.x - b.x);
}

// ---------- NEW: direction printer helpers ----------
int dirBetween(Cell a, Cell b) {
  int dy = b.y - a.y, dx = b.x - a.x;
  for (int d = 0; d < 4; ++d) {
    if (dy == DIRS[d][0] && dx == DIRS[d][1]) return d;
  }
  return -1;
}

void printMoveScript(const Cell *path, int pathLen, int startDir) {
  int dir = startDir;
  Serial.println("\n🧭 Turn-by-turn script:");
  for (int i = 0; i < pathLen - 1; ++i) {
    Cell a = path[i];
    Cell b = path[i + 1];
    int stepDir = dirBetween(a, b);
    if (stepDir < 0) {
      Serial.print("⚠️  Non-adjacent step: (");
      Serial.print(a.y);
      Serial.print(",");
      Serial.print(a.x);
      Serial.print(") -> (");
      Serial.print(b.y);
      Serial.print(",");
      Serial.print(b.x);
      Serial.println(")");
      continue;
    }

    Serial.print("from (");
    Serial.print(a.y);
    Serial.print(",");
    Serial.print(a.x);
    Serial.print(") to (");
    Serial.print(b.y);
    Serial.print(",");
    Serial.print(b.x);
    Serial.print("): ");

    int diff = (stepDir - dir + 4) % 4;
    if (diff == 0) {
      //Serial.print("no turn, ");
    } else if (diff == 1) {
      //Serial.print("turn RIGHT 90°, ");
      turnright();
    } else if (diff == 2) {
      //Serial.print("turn 180°, ");
      turnback();
    } else {  // diff == 3
      //Serial.print("turn LEFT 90°, ");
      turnleft();
    }

    //Serial.println("then FORWARD 1 cell");
    forward();

    dir = stepDir;
  }
  Serial.print("Final facing: ");
  run_end_dir = dir;
  Serial.println(DIR_NAMES[dir]);
}

// The A* you requested (kept in your style & signature)
bool DFS_a_star_path(Cell start, Cell goal, int init_dir, bool explored_mask[MAZE_SIZE][MAZE_SIZE]) {
  ListN open;
  DFS_listn_init(open);
  Node closed[MAZE_SIZE * MAZE_SIZE * 2];
  int closedN = 0;
  bool used[MAZE_SIZE][MAZE_SIZE];
  for (int y = 0; y < MAZE_SIZE; ++y)
    for (int x = 0; x < MAZE_SIZE; ++x) used[y][x] = false;

  Node s = { start, 0, DFS_manhattan(start, goal), -1 };
  DFS_listn_add(open, s);

  while (open.n > 0) {
    int idx = DFS_listn_findMinF(open);
    Node cur = DFS_listn_takeAt(open, idx);

    if (DFS_sameCell(cur.c, goal)) {
      // stash the current in closed so parentIdx chain is valid
      closed[closedN] = cur;
      int at = closedN++;

      // reconstruct reversed
      Cell rev[MAZE_SIZE * MAZE_SIZE * 2];
      int rn = 0;
      int i = at;
      while (i >= 0) {
        rev[rn++] = closed[i].c;
        i = closed[i].parentIdx;
      }

      // forward path
      Cell fwd[MAZE_SIZE * MAZE_SIZE * 2];
      int fn = 0;
      for (int k = rn - 1; k >= 0; --k) fwd[fn++] = rev[k];

      // print cell list
      Serial.println();
      Serial.print("Shortest Path: ");
      for (int k = 0; k < fn; ++k) {
        if (k) Serial.print(" -> ");
        Serial.print("(");
        Serial.print(fwd[k].y);
        Serial.print(",");
        Serial.print(fwd[k].x);
        Serial.print(")");
      }
      Serial.println();

      printMoveScript(fwd, fn, init_dir);

      return true;
    }

    if (used[cur.c.y][cur.c.x]) continue;
    used[cur.c.y][cur.c.x] = true;

    int myIndex = closedN;
    closed[closedN++] = cur;

    for (int d = 0; d < 4; ++d) {
      int ny = cur.c.y + DIRS[d][0], nx = cur.c.x + DIRS[d][1];
      if (!DFS_in_bounds(ny, nx)) continue;
      // must not be a wall along this edge
      if (mazeArr[cur.c.y][cur.c.x][d] == 1) continue;
      // only step onto explored cells
      if (!explored_mask[ny][nx]) continue;
      if (used[ny][nx]) continue;

      Node nxt = { { ny, nx }, cur.g + 1, cur.g + 1 + DFS_manhattan({ ny, nx }, goal), myIndex };
      DFS_listn_add(open, nxt);
    }
  }

  Serial.println();
  Serial.print("No path found to (");
  Serial.print(goal.y);
  Serial.print(",");
  Serial.print(goal.x);
  Serial.println(")");
  return false;
}

// ====== EEPROM persistence (header + CRC16) ======
struct MazeHeader {
  uint32_t magic;    // 'MAZE' = 0x4D415A45
  uint16_t version;  // bump if you change format
  uint16_t rows;     // MAZE_SIZE
  uint16_t cols;     // MAZE_SIZE
  uint16_t bytes;    // payload size
  uint16_t crc;      // CRC16 over payload
};

static const uint32_t MAZE_MAGIC = 0x4D415A45UL;
static const uint16_t MAZE_VERSION = 1;
static const int EEPROM_BASE = 0;  // start at 0

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  data ^= (uint8_t)(crc & 0xFF);
  data ^= data << 4;
  return (uint16_t)((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}
static uint16_t crc16_bytes(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) crc = crc16_update(crc, p[i]);
  return crc;
}

bool saveMazeToEEPROM() {
  MazeHeader hdr;
  hdr.magic = MAZE_MAGIC;
  hdr.version = MAZE_VERSION;
  hdr.rows = MAZE_SIZE;
  hdr.cols = MAZE_SIZE;
  hdr.bytes = (uint16_t)sizeof(mazeArr);
  hdr.crc = crc16_bytes(reinterpret_cast<const uint8_t *>(mazeArr), sizeof(mazeArr));

  int need = (int)sizeof(MazeHeader) + (int)sizeof(mazeArr);
  if (EEPROM.length() < EEPROM_BASE + need) {
    Serial.println(F("[EEPROM] Not enough space.]"));
    return false;
  }
  int addr = EEPROM_BASE;
  EEPROM.put(addr, hdr);
  addr += sizeof(MazeHeader);
  EEPROM.put(addr, mazeArr);
  addr += sizeof(mazeArr);
  Serial.print(F("[EEPROM] Saved bytes="));
  Serial.print(hdr.bytes);
  Serial.print(F(" CRC=0x"));
  Serial.println(hdr.crc, HEX);
  return true;
}

bool loadMazeFromEEPROM() {
  if (EEPROM.length() < EEPROM_BASE + (int)sizeof(MazeHeader)) return false;

  int addr = EEPROM_BASE;
  MazeHeader hdr;
  EEPROM.get(addr, hdr);
  addr += sizeof(MazeHeader);

  if (hdr.magic != MAZE_MAGIC || hdr.version != MAZE_VERSION) return false;
  if (hdr.rows != MAZE_SIZE || hdr.cols != MAZE_SIZE) {
    Serial.println(F("[EEPROM] Size mismatch."));
    return false;
  }
  if (hdr.bytes != sizeof(mazeArr)) {
    Serial.println(F("[EEPROM] Data size mismatch."));
    return false;
  }
  if (EEPROM.length() < EEPROM_BASE + (int)sizeof(MazeHeader) + (int)hdr.bytes) return false;

  // Read to temp, CRC, then copy
  static int8_t tmp[MAZE_SIZE][MAZE_SIZE][4];
  EEPROM.get(addr, tmp);

  uint16_t calc = crc16_bytes(reinterpret_cast<const uint8_t *>(tmp), sizeof(tmp));
  if (calc != hdr.crc) {
    Serial.print(F("[EEPROM] CRC mismatch: file=0x"));
    Serial.print(hdr.crc, HEX);
    Serial.print(F(" calc=0x"));
    Serial.println(calc, HEX);
    return false;
  }
  memcpy(mazeArr, tmp, sizeof(tmp));
  Serial.print(F("[EEPROM] Loaded bytes="));
  Serial.print(hdr.bytes);
  Serial.print(F(" CRC=0x"));
  Serial.println(calc, HEX);
  return true;
}

void clearSavedMaze() {
  MazeHeader blank{};
  EEPROM.put(EEPROM_BASE, blank);
  Serial.println(F("[EEPROM] Cleared saved maze header."));
}

// ===== Utilities =====
void initMazeUnknown() {
  for (int y = 0; y < MAZE_SIZE; ++y)
    for (int x = 0; x < MAZE_SIZE; ++x)
      for (int d = 0; d < 4; ++d)
        mazeArr[y][x][d] = -1;
}

void printMaze() {
  Serial.println(F("=== Maze [N,E,S,W] per cell (-1=?, 0=open, 1=wall) ==="));
  for (int y = 0; y < MAZE_SIZE; ++y) {
    for (int x = 0; x < MAZE_SIZE; ++x) {
      Serial.print("[");
      for (int d = 0; d < 4; ++d) {
        Serial.print(mazeArr[y][x][d]);
        if (d < 3) Serial.print(",");
      }
      Serial.print("] ");
    }
    Serial.println();
  }
}

void printMenu() {
  Serial.println();
  Serial.println(F("=== Menu ==="));
  Serial.println(F("e = Explore now (and save)"));
  Serial.println(F("p = Print current maze"));
  Serial.println(F("l = Load saved maze from EEPROM"));
  Serial.println(F("c = Clear saved maze header"));
  Serial.println(F("a = A* shortest path to a goal (after explore)"));
  Serial.print(F("> "));
}

// ===== Arduino setup/loop =====
void setup() {
  Serial.begin(115200);
  
  pinMode(DIP1, INPUT_PULLUP);

  int state1 = digitalRead(DIP1);  // invert so ON=1, OFF=0

  if (state1 == LOW) {
    MODE = RUN;
  }

  Serial.println("MODEEEEEEEEEEEEEEEEEEE ");

  Serial.println(MODE);

  pinMode(frontRightTOF, INPUT);
  pinMode(frontLeftTOF, INPUT);

  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  pinMode(leftMotor_pin1, OUTPUT);
  pinMode(leftMotor_pin2, OUTPUT);
  pinMode(rightMotor_pin1, OUTPUT);
  pinMode(rightMotor_pin2, OUTPUT);

  stopMotors();

  pinMode(leftEncoderPinA, INPUT_PULLUP);
  pinMode(leftEncoderPinB, INPUT_PULLUP);
  pinMode(rightEncoderPinA, INPUT_PULLUP);
  pinMode(rightEncoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), handleRightEncoder, CHANGE);


  Wire.begin();
  delay(1000);
  Serial.println("Starting...");

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  Serial.println("ICM20602 Ready.");
  Wire.endTransmission();

  // Initialize XSHUT pins
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  // Turn off both sensors
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);
  // Turn on sensor1 only and set its address
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setAddress(SENSOR1_ADDR);
  // Turn on sensor2 only and set its address
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setAddress(SENSOR2_ADDR);
  // Start continuous ranging mode
  sensor1.startContinuous();
  sensor2.startContinuous();

  lastTime = millis();

  updateYaw();
  delay(1000);


  // delay(1000);
  Serial.println("start");

  // String ins = "FFFRFRFFLF";
  // // String ins = "FFFFFRFRFLFRFFLFLF";
  // // String ins = "FLF";

  // for (int i = 0; i < ins.length(); i++) {
  //   Serial.println(ins[i]);  // prints each character
  //   Serial.println("INSTRUCTION");
  //   if (ins[i] == 'F') {
  //     moveOneCellForward(getEncoderCountFromDistance(19.2));
  //   } else if (ins[i] == 'L') {
  //     Serial.println("Turn left");

  //     turnLeft_PID(88);

  //   } else if (ins[i] == 'R') {
  //     Serial.println("Turn right");

  //     turnLeft_PID(-88);
  //   }
  //   stopMotors();
  // }
  // stopMotors();

  // while (!Serial) { /* wait for USB */
  // }

  Serial.println("\n=== Physical DFS Maze Explorer + EEPROM ===");
  Serial.println("Trying to load saved maze...");

  if (loadMazeFromEEPROM()) {
    Serial.println(F("[EEPROM] Using stored maze."));
  } else {
    Serial.println(F("[EEPROM] No valid saved maze found. Initializing to unknown."));
    initMazeUnknown();
  }

  printMaze();
  printMenu();
}

void drive(long targetTicks) {
  leftEncoder_count = 0;
  rightEncoder_count = 0;
  lastLeftCorrectionLeft = 0;
  lastLeftCorrectionRight = 0;
  lastError = 0;
  integral = 0;

  updateYaw();
  float targetYaw = yaw;

  analogWrite(leftMotor_pin2, 0);
  analogWrite(rightMotor_pin2, 0);
  analogWrite(leftMotor_pin1, leftSpeed);
  analogWrite(rightMotor_pin1, rightSpeed);


  while (true) {
    // Serial.print(leftEncoder_count);
    // Serial.print(" ");
    // Serial.println(rightEncoder_count);
    long left = abs(leftEncoder_count);
    long right = abs(rightEncoder_count);
    if (left >= targetTicks && right >= targetTicks) {
      stopMotors();
      // Serial.println("Encoder turn completed.");
      break;
    }

    encoder_PID();
    // wall_PID();
    // left_wall_PID();
    // moveForward_GYRO_PID();
  }
}

int getEncoderCountForturn(int angle) {
  return (int)((9.1 / 2.0) * (angle / 180) * (encoderCPR / 3.4));
}

int getEncoderCountFromDistance(float dist) {
  return (int)(((encoderCPR / (PI * 3.4)) * (dist)) - CORRECTION_OFFSET);
}

void loop() {
  if (MODE == SEARCH) {
    // Serial.println(F("\n[RUN] Exploring..."));
    Point path[MAZE_SIZE * MAZE_SIZE * 8];
    bool explored[MAZE_SIZE][MAZE_SIZE];
    initMazeUnknown();
    (void)explorePhysical(0, 0, initial_direction, path, explored);
    saveMazeToEEPROM();
    delay(1000);
    // Serial.println(F("[INFO] Exploration complete. You can now press 'a' to run A* to a goal."));

    static bool explored_mask[MAZE_SIZE][MAZE_SIZE];
    for (int y = 0; y < MAZE_SIZE; ++y) {
      for (int x = 0; x < MAZE_SIZE; ++x) {
        bool known = false;
        for (int d = 0; d < 4; ++d) {
          if (mazeArr[y][x][d] != -1) {
            known = true;
            break;
          }
        }
        explored_mask[y][x] = known;
      }
    }

    Cell expl_end{ expl_end_y, expl_end_x };
    (void)DFS_a_star_path(expl_end, start_, expl_end_dir, explored_mask);
  } else if (MODE == RUN) {
    // Build an explored mask by considering any cell that has at least one non-unknown edge
    loadMazeFromEEPROM();
    printMaze();
    static bool explored_mask[MAZE_SIZE][MAZE_SIZE];
    for (int y = 0; y < MAZE_SIZE; ++y) {
      for (int x = 0; x < MAZE_SIZE; ++x) {
        bool known = false;
        for (int d = 0; d < 4; ++d) {
          if (mazeArr[y][x][d] != -1) {
            known = true;
            break;
          }
        }
        explored_mask[y][x] = known;
      }
    }

    Cell goal1{ 7, 7 };
    Cell goal2{ 7, 8 };
    Cell goal3{ 8, 7 };
    Cell goal4{ 8, 8 };

    // Cell goal1{ 1, 2 };
    // Cell goal2{ 2, 1 };
    // Cell goal3{ 2, 2 };
    // Cell goal4{ 1, 1 };

    Cell goal_;
    bool is_possible = DFS_a_star_path(start_, goal1, initial_direction, explored_mask);
    goal_ = goal1;
    if (!is_possible) {
      is_possible = DFS_a_star_path(start_, goal2, initial_direction, explored_mask);
      goal_ = goal2;
      if (!is_possible) {
        is_possible = DFS_a_star_path(start_, goal3, initial_direction, explored_mask);
        goal_ = goal3;
        if (!is_possible) {
          is_possible = DFS_a_star_path(start_, goal4, initial_direction, explored_mask);
          goal_ = goal4;
        }
      }
    }

    delay(3000);
    (void)DFS_a_star_path(goal_, start_, run_end_dir, explored_mask);
  }
}