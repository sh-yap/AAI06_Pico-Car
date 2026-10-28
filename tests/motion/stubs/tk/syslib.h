/* Encoder snapshot tests do not initialise board interrupts. */
#define DI(state) do { (state) = 0U; } while (0)
#define EI(state) do { (void)(state); } while (0)
void ClearInt(UINT intno);
void EnableInt(UINT intno, INT level);
