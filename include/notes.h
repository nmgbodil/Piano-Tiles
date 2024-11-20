#define NUM_NOTES 26

typedef struct {
    int start_time;  // Duration (how long the note should be displayed)
    int color;     // Color for the tile
} Note;

// Active notes' positions
int y_positions[NUM_NOTES];
int x_positions[NUM_NOTES];

// Match colors to multiplier
int mult_to_color[3] = {0xF800, 0x0000, 0xFD20};

Note song_notes[NUM_NOTES] = {
    {0, 0xF800},    // E
    {500, 0x07E0},  // D
    {1000, 0x001F}, // C
    {1500, 0x07E0}, // D
    {2000, 0xF800}, // E
    {2500, 0xF800}, // E
    {3000, 0xF800}, // E
    {4000, 0x07E0}, // D
    {4500, 0x07E0}, // D
    {5000, 0x07E0}, // D
    {6000, 0xF800}, // E
    {6500, 0xFFE0}, // G
    {7000, 0xFFE0}, // G
    {8000, 0xF800}, // E
    {8500, 0x07E0}, // D
    {9000, 0x001F}, // C
    {9500, 0x07E0}, // D
    {10000, 0xF800}, // E
    {10500, 0xF800}, // E
    {11000, 0xF800}, // E
    {12000, 0xF800}, // E
    {12500, 0x07E0}, // D
    {13000, 0x07E0}, // D
    {13500, 0xF800}, // E
    {14000, 0x07E0}, // D
    {14500, 0x001F}  // C
};
