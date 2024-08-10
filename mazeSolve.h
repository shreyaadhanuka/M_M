#include <utility>            // std::pair
#include <queue>              //std::queue
#include <vector>
using namespace std;

Position pos(0, 0, 0);

int orient=0;
int identifyBlockType() {
  uint8_t a[4], b[4];
  int error = 30;
  a[0] = sensor3.readRangeSingleMillimeters() < 250;
  a[1] = sensor1.readRangeSingleMillimeters() < 250;
  a[3] = sensor2.readRangeSingleMillimeters() < 250;
  a[2] = 0;

  for (int i = 0; i < 4; i++) {
    b[i] = a[(i + orient) % 4];
  }

  uint8_t type = 8 * b[0] + 4 * b[1] + 2 * b[2] + b[3];
  // print("Block (", pos.x, ", ", pos.y, ") is of type: ", type);
  maze[pos.x][pos.y] = type;
  return type;
}

void printMatrix(uint8_t mat[][MAZE_SIZE]) {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      Serial.print(mat[i][j]);
      Serial.print(' ');
    }
    Serial.println();
  }
}

vector<pair<uint8_t, uint8_t>> neighbours(uint8_t i, uint8_t j) {
  vector<pair<uint8_t, uint8_t>> nb;
  if (i > 0 && flood[i - 1][j] == 255) {
    if (((maze[i - 1][j] >> 1) & 1 || (maze[i][j] >> 3) & 1) == 0)
      nb.push_back({ i - 1, j });
  }
  if (i < MAZE_SIZE - 1 && flood[i + 1][j] == 255) {
    if (((maze[i + 1][j] >> 3) & 1 || (maze[i][j] >> 1) & 1) == 0)
      nb.push_back({ i + 1, j });
  }
  if (j > 0 && flood[i][j - 1] == 255) {
    if (((maze[i][j - 1] >> 2) & 1 || (maze[i][j]) & 1) == 0)
      nb.push_back({ i, j - 1 });
  }
  if (j < MAZE_SIZE - 1 && flood[i][j + 1] == 255) {
    if (((maze[i][j + 1]) & 1 || (maze[i][j] >> 2) & 1) == 0)
      nb.push_back({ i, j + 1 });
  }
  return nb;
}


void floodfill() {
  int b = MAZE_SIZE / 2;
  int a = b - 1;
  memset(flood, -1, MAZE_SIZE * MAZE_SIZE * sizeof(uint8_t));
  memset(&flood[a][a], 0, 2 * sizeof(uint8_t));
  memset(&flood[b][a], 0, 2 * sizeof(uint8_t));

  queue<pair<uint8_t, uint8_t>> q;
  // printMatrix(flood);

  q.push({ a, a });
  q.push({ a, b });
  q.push({ b, a });
  q.push({ b, b });

  while (!q.empty()) {
    auto [i, j] = q.front();
    q.pop();

    for (auto [a, b] : neighbours(i, j)) {
      if (flood[a][b] == 255) {
        flood[a][b] = flood[i][j] + 1;
        q.push({ a, b });
      }
    }
  }
}

int nextBlock() {
  int i = pos.x, j = pos.y;
  if (i > 0 && flood[i - 1][j] < flood[i][j]) {
    if (((maze[i - 1][j] >> 1) & 1 || (maze[i][j] >> 3) & 1) == 0)
      return 0;
  }
  if (i < MAZE_SIZE && flood[i + 1][j] < flood[i][j]) {
    if (((maze[i + 1][j] >> 3) & 1 || (maze[i][j] >> 1) & 1) == 0)
      return 2;
  }
  if (j > 0 && flood[i][j - 1] < flood[i][j]) {
    if (((maze[i][j - 1] >> 2) & 1 || (maze[i][j]) & 1) == 0)
      return 1;
  }
  if (j < MAZE_SIZE && flood[i][j + 1] < flood[i][j]) {
    if (((maze[i][j + 1]) & 1 || (maze[i][j] >> 2) & 1) == 0)
      return 3;
  }
  return -1;
}