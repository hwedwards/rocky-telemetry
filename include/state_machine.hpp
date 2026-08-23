enum struct states {
    off, 
    logging, 
}; 
class StateMachine 
{
    Public:
        // Constructor
        StateMachine() : current_state(states::off) {}
        // Check the current state
        states get_current_state() const {
            return current_state;
        }
        // Transition to the next state based on the current state
        void transition() {
            switch (current_state) {
                case states::off:
                    current_state = states::logging;
                    break;
                case states::logging:
                    current_state = states::off;
                    break;
            }
        }
    Private:
        states current_state;    
}; 