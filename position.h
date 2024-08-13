class Position {
private:
  uint8_t _MAZE_SIZE;
public:
  uint8_t x;
  uint8_t y;
  uint8_t orient;

  Position();
  Position(uint8_t x, uint8_t y, uint8_t orient);
  void rotateLeft();
  void rotateRight();
  void rotate180();
  void moveTowards(int newOrient);
  void forward();
  void state() const;
};

Position::Position() {
  x = 0;
  y = 0;
  orient = 0;
  _MAZE_SIZE = 10;  // Default MAZE_SIZE
}

Position::Position(uint8_t x, uint8_t y, uint8_t orient) {
  this->x = x;
  this->y = y;
  this->orient = orient;
  this->_MAZE_SIZE = MAZE_SIZE;
}

void Position::rotateLeft() {
  Serial.print("rotating left");
  orient = (orient + 1) % 4;
  rotate(-90); //call rotate function here
}

void Position::rotateRight() {
  Serial.print("rotating right");
  orient = (orient + 3) % 4;
  rotate(90); //call rotate function here
}

void Position::rotate180() {
  Serial.print("rotating 180");
  orient = (orient + 2) % 4;
  rotate(180); //call rotate function here
}

void Position::moveTowards(int newOrient) {
  int diff = newOrient - orient;
  if (diff == 1 || diff == -3) rotateLeft();
  else if (diff == -1 || diff == 3) rotateRight();
  else if (abs(diff) == 2) rotate180();
}

void Position::forward() {
  if (orient == 0 && x > 0) x--;
  if (orient == 1 && y > 0) y--;
  if (orient == 2 && x < _MAZE_SIZE - 1) x++;
  if (orient == 3 && y < _MAZE_SIZE - 1) y++;
}
