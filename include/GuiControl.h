#ifndef GUICONTROL_H
#define GUICONTROL_H

#include "SnakeSimDefines.h" // include this last
#include <Robot.h>

#ifdef USE_IRR
#include <irrlicht.h>
namespace chrono {
namespace irrlicht {
class ChIrrApp;
}
}

//********************
// EVENT RECEIVER CLASS
//********************
class MyEventReceiver : public irr::IEventReceiver {
private:
  bool KeyIsDown;
  chrono::irrlicht::ChIrrApp *mApp;
  irr::gui::IGUICheckBox *mChBoxPauseSim;
  irr::gui::IGUIEditBox *mEdBoxTimeStep;
  irr::gui::IGUIEditBox *mEdBox_k;
  irr::gui::IGUIEditBox *mEdBox_A;
  irr::gui::IGUIEditBox *mEdBox_w;
  irr::gui::IGUIEditBox *mEdBox_h;
  GlobalControlSet *mControlSet;

public:
  MyEventReceiver(chrono::irrlicht::ChIrrApp *, GlobalControlSet *);
  bool OnEvent(const irr::SEvent &event);
  // update text when global params are changed
  void UpdateText();
};

#endif

#endif
