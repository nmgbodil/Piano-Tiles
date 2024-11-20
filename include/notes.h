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

/*
#define NOTE_C 0x001F  // Blue
#define NOTE_D 0x07E0  // Green
#define NOTE_E 0xF800  // Red
#define NOTE_F 0xF81F  // Purple
#define NOTE_G 0xFFE0  // Yellow
#define NOTE_A 0xFD20  // Orange
#define NOTE_B 0x07FF  // Cyan
#define REST   0x0000  // Black for rest

// mary had a little lamb
#define NUM_NOTES_MARY 26

typedef struct {
    int start_time;  // When the note starts
    int color;       // Color representing the note
} Note;


int y_positions_mary[NUM_NOTES_MARY];
int x_positions_mary[NUM_NOTES_MARY];

Note mary_had_a_little_lamb[NUM_NOTES_MARY] = {
    {0,    NOTE_E},  // E
    {500,  NOTE_D},  // D
    {1000, NOTE_C},  // C
    {1500, NOTE_D},  // D
    {2000, NOTE_E},  // E
    {2500, NOTE_E},  // E
    {3000, NOTE_E},  // E
    {4000, NOTE_D},  // D
    {4500, NOTE_D},  // D
    {5000, NOTE_D},  // D
    {5500, NOTE_E},  // E
    {6000, NOTE_G},  // G
    {6500, NOTE_G},  // G
    {7000, NOTE_E},  // E
    {7500, NOTE_D},  // D
    {8000, NOTE_C},  // C
    {8500, NOTE_D},  // D
    {9000, NOTE_E},  // E
    {9500, NOTE_E},  // E
    {10000,NOTE_E},  // E
    {10500,NOTE_E},  // E
    {11000,NOTE_D},  // D
    {11500,NOTE_D},  // D
    {12000,NOTE_E},  // E
    {12500,NOTE_D},  // D
    {13000,NOTE_C}   // C
};

// twinkle twinkle little star
#define NUM_NOTES_TWINKLE 42

int y_positions_twinkle[NUM_NOTES_TWINKLE];
int x_positions_twinkle[NUM_NOTES_TWINKLE];

Note twinkle_twinkle[NUM_NOTES_TWINKLE] = {
    {0,    NOTE_C},   // C
    {500,  NOTE_C},   // C
    {1000, NOTE_G},   // G
    {1500, NOTE_G},   // G
    {2000, NOTE_A},   // A
    {2500, NOTE_A},   // A
    {3000, NOTE_G},   // G
    {3500, REST},     // Rest
    {4000, NOTE_F},   // F
    {4500, NOTE_F},   // F
    {5000, NOTE_E},   // E
    {5500, NOTE_E},   // E
    {6000, NOTE_D},   // D
    {6500, NOTE_D},   // D
    {7000, NOTE_C},   // C
    {7500, REST},     // Rest
    {8000, NOTE_G},   // G
    {8500, NOTE_G},   // G
    {9000, NOTE_F},   // F
    {9500, NOTE_F},   // F
    {10000,NOTE_E},   // E
    {10500,NOTE_E},   // E
    {11000,NOTE_D},   // D
    {11500,REST},     // Rest
    {12000,NOTE_G},   // G
    {12500,NOTE_G},   // G
    {13000,NOTE_F},   // F
    {13500,NOTE_F},   // F
    {14000,NOTE_E},   // E
    {14500,NOTE_E},   // E
    {15000,NOTE_D},   // D
    {15500,REST},     // Rest
    {16000,NOTE_C},   // C
    {16500,NOTE_C},   // C
    {17000,NOTE_G},   // G
    {17500,NOTE_G},   // G
    {18000,NOTE_A},   // A
    {18500,NOTE_A},   // A
    {19000,NOTE_G},   // G
    {19500,REST},     // Rest
    {20000,NOTE_F},   // F
    {20500,NOTE_F},   // F
    {21000,NOTE_E},   // E
    {21500,NOTE_E},   // E
    {22000,NOTE_D},   // D
    {22500,NOTE_D},   // D
    {23000,NOTE_C}    // C
};

// row, row, row your boat
#define NUM_NOTES_ROW 20

int y_positions_row[NUM_NOTES_ROW];
int x_positions_row[NUM_NOTES_ROW];

Note row_row_row_your_boat[NUM_NOTES_ROW] = {
    {0,    NOTE_C},   // C
    {500,  NOTE_C},   // C
    {1000, NOTE_C},   // C
    {1500, NOTE_D},   // D
    {2000, NOTE_E},   // E
    {2500, NOTE_E},   // E
    {3000, NOTE_D},   // D
    {3500, NOTE_E},   // E
    {4000, NOTE_F},   // F
    {4500, NOTE_G},   // G
    {5000, NOTE_G},   // G
    {5500, NOTE_G},   // G
    {6000, NOTE_G},   // G
    {6500, NOTE_F},   // F
    {7000, NOTE_E},   // E
    {7500, NOTE_C},   // C
    {8000, NOTE_C},   // C
    {8500, NOTE_G},   // G
    {9000, NOTE_G},   // G
    {9500, NOTE_F},   // F
    {10000,NOTE_E},   // E
    {10500,NOTE_D},   // D
    {11000,NOTE_C}    // C
};

// itsy bitsy spider
#define NUM_NOTES_ITSY 32

int y_positions_itsy[NUM_NOTES_ITSY];
int x_positions_itsy[NUM_NOTES_ITSY];

Note itsy_bitsy_spider[NUM_NOTES_ITSY] = {
    {0,    NOTE_C},   // C
    {500,  NOTE_D},   // D
    {1000, NOTE_E},   // E
    {1500, NOTE_F},   // F
    {2000, NOTE_E},   // E
    {2500, NOTE_D},   // D
    {3000, NOTE_C},   // C
    {3500, NOTE_E},   // E
    {4000, NOTE_F},   // F
    {4500, NOTE_G},   // G
    {5000, NOTE_C},   // C 
    {5500, NOTE_C},   // C
    {6000, NOTE_G},   // G
    {6500, NOTE_F},   // F
    {7000, NOTE_E},   // E
    {7500, NOTE_C},   // C
    {8000, NOTE_C},   // C
    {8500, NOTE_D},   // D
    {9000, NOTE_E},   // E
    {9500, NOTE_F},   // F
    {10000,NOTE_E},   // E
    {10500,NOTE_D},   // D
    {11000,NOTE_C},   // C
    {11500,NOTE_E},   // E
    {12000,NOTE_D},   // D
    {12500,NOTE_C},   // C
    {13000,NOTE_D},   // D
    {13500,NOTE_G},   // G
    {14000,NOTE_G},   // G
    {14500,NOTE_C},   // C
    {15000,NOTE_C},   // C
    {15500,NOTE_C},   // C
    {16000,NOTE_C}    // C 
};
*/