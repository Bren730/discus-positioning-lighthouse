#include <Arduino.h>

class DataDiscus {

  

  private:

    // Functions

  public:

  enum State {STATE_DISCONNECTED, STATE_CONNECTED, STATE_PAIRING};

  // Empty constructor
  DataDiscus();

    // Variables

    // Definitions

    // Functions

    // Enumerators
    State state = State::STATE_DISCONNECTED;

};
