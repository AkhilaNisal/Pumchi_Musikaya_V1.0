int current_pos[3] = { 0, 0, 0 };  // x,y,orient

void Move(int move_pos[2]) {
  int n_orient;   

  int y = move_pos[1] - current_pos[1];
  if (y == 1) {
    n_orient = 0;
  } else if (y == -1) {
    n_orient = 2;
  } else {
    int x = move_pos[0] - current_pos[0];
    if (x == 1) {
      n_orient = 1;
    } else if (x == -1) {
      n_orient = 3;
    } else {
      n_orient = current_pos[2];  
    }
  }

  int dir = n_orient - current_pos[2]; 
  if (dir == -1) {
    //turn right 90 degrees
  } else if (dir == 0) {
    // move fwd
  } else if (dir == 1) {
    // turn left 90 n move fwd
  } else if (dir == 2) {
    // turn left 180 n move fwd
  }

  // update orientation
  current_pos[2] = n_orient;
}

int dirToNum(int dir[2]) {
  if (dir[0] == -1 && dir[1] == 0) {
    return 0;
  } else if (dir[0] == 0 && dir[1] == 1) {
    return 1;
  } else if (dir[0] == 1 && dir[1] == 0) {
    return 2;
  } else {
    return 3;
  }
}
