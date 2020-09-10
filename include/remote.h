#ifndef _REMOTE_H_
#define _REMOTE_H_

class TwoAxisControl
{
public:
  char x;
  char y;
};

class Remote
{
public:
  TwoAxisControl axis1;
  TwoAxisControl axis2;

  void setup();
};

#endif
