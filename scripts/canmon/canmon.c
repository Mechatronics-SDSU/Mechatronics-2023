///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Connor Larmer                                                             //
// 15 June 2023	                                                             //
// SDSU MECHATRONICS CAN MONITOR                                             //
// The purpose of this program is to provide                                 //
// a low-overhead method of monitoring all                                   //
// data passing through Percy's CAN bus without sacrificing                  //
// ease-of-use or  `*~aesthetic~`*                                           //
//                                                                           //
// Yea its all in one file, cry about it.                                    //
// !!! VERY IMPORTANT: SEE BELOW FOR CONFIG !!!								 //
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>

///////////////////////////////
// CAN SPECIFIC HEADERS
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
///////////////////////////////

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// CONFIG //

enum HW_DEV_ID
{
	ID_EMBEDSYS = 0x0000,		// HW ID for EMBEDSYS
	ID_PWRSYS = 0x0001,			// HW ID for PWRSYS
	ID_WAYFDVL = 0x0002,		// HW ID for WAYFDVL	
	ID_MS5837 = 0x0003,			// HW ID for MS5837
	ID_BRLIGHT = 0x0004,		// HW ID for BRLIGHT
};

const float TICK_INTERVAL = 250;	// HOW FAST TO UPDATE DISPLAY (MILLISECONDS)
const char SHOW_MOTOR_AILIAS = 1;	// SHOW MOTOR NAMES (SEE BELOW)
const char MOTOR_ALIAS[8][24] =		// MOTOR NAMES (SHOULD BE CONFIGURED)
{
	"0 FRONT LEFT    (XY)",	
	"1 FRONT RIGHT   (XY)",	
	"2 FRONT LEFT    (Z)",	
	"3 FRONT RIGHT   (Z)",	
	"4 REAR  LEFT    (XY)",	
	"5 REAR  RIGHT   (XY)",	
	"6 REAR  LEFT    (Z)",	
	"7 REAR  RIGHT   (Z)",	
};

// CAN BUS TO OPEN !!! SHOULD BE CHANGED !!!
const char* CAN_BUS_INTERFACE = "vcan0";	

// Don't worry about these warnings lol
// These are just some can frames, nothing to be done here!
struct can_frame STOP_FRAME = { 0x000, 0, (u_int8_t){0} };
struct can_frame AC_FRAME 	= { 0x00A, 0, (u_int8_t){0} };
struct can_frame MOT_FRAME 	= { 0x010, 8, (u_int8_t){0} };

// You probably don't need to change these
char* SCRIPT_DIR = ".canmon";	// Script directory name (/home/user/[SCRIPT_DIR])
char* SCRIPT_EXT = ".sh";		// script filetype

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// This is where the actual code starts

/*
 * struct frame_data
 *
 *	This is used during the parsing of can frames as a
 *	quick way to split up and hold all the datapoints
 * 	that a frame describes.
 */
struct frame_data
{
	u_int8_t id;	
	u_int16_t dev;	
	u_int16_t topic;	
	u_int32_t data;	
};

/*
 * struct canmon
 *	
 *	This is basically the program's internal configuration settings
 *	it is used to hold bits of data that are frequently passed around
 * 	between functions or to track what mode/state the program is currently
 *	operating in.
 */
struct
{
	int can_bus;
	struct can_frame frame;
	struct frame_data data;
	struct termios tty;
	int screen_w;
	int screen_h;
	char last_action[32];
	char can_error;
	char script_error;
	char script_mode_en;
	char script_idx;
} canmon;

/*
 * struct scripts
 *
 *	Similar to canmon, this structure holds almost all automation
 * 	related variables. This includes file descriptors, a list of
 * 	available scripts, and a 4 line output buffer to store anything
 * 	a script writes to STDOUT
 */
struct
{
	char dir[32];
	DIR *d_fd;
	struct dirent* scripts;
	int n;
	FILE *s_fd;
	char s_buf[4][64];
} scripts;

/*
 * struct hw_data
 *
 * 	Not much of interest here, this simply allows for quick
 * 	access to the latest decoded sensor/motor data from the
 * 	CAN handler. I know global variables are a nono, but c'mon,
 * 	this program is a JOKE.
 */
struct
{
	// MOTOR //
	int motor_val[8];
	// DVL //
	char dvl_stat;
	float dvl_vel_x;		
	float dvl_vel_y;		
	float dvl_vel_z;		
	float dvl_vel_e;		
	float dvl_dist;
	// MS5837 //
	char ms5837_stat;
	float ms5837_depth;
	float ms5837_temp;
	// BRLIGHT //
	char brlight_stat;
	float brlight_bright;
} hw_data;

/*
 * struct screen
 *
 *	You may have noticed that this is a TUI program, however
 * 	for some reason it doesn't use ncurses (why???). This structure
 * 	allows all functions to quickly access the 'screen' buffer.
 * 	ASCII characters are written to it as pixels would be written
 * 	to a screen (x,y). This is one of the few dynamically-allocated
 * 	components of the program.
 */
struct
{
	int w;
	int h;
	int len;
	char* buf;
} screen;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Function prototypes. Note that there are some functions not present here.

///////////////////////////////////////////////////////////
// Automation
int init_script();
int exec_script();
///////////////////////////////////////////////////////////
// CAN //
int init_can(int* can_sock);
int send_can(int* can_sock, struct can_frame* frame);
int recv_can(int* can_sock, struct can_frame* frame);
///////////////////////////////////////////////////////////
// PROGRAM CTL //
void render();
void clear();
void kill();
void handle_input(char* c);
void handle_can();
///////////////////////////////////////////////////////////
// GRAPHICS AND BUFFERING //
void newline();
void draw_hline(int y);
void draw_vline(int x, int y, int l);
void draw_to_buf();
void draw_slider(int x, int y, int val, int min, int max, int w);
void init_screen();
void writeb(int x, int y, char* c, int len);
char* buf_access(int x, int y);
void draw_motors(int x, int y, int motors[], int len, char center);
void draw_ms5837(int x, int y);
void draw_dvl(int x, int y);
void draw_scripts(int x, int y);
void draw_script_buf(int x, int y);
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Program start point and execution loop
int main()
{
	init_can(&canmon.can_bus);
	atexit(kill); // clean up after yourself :)

	/////////////////////////////////////////
	// Terminal settings
	tcgetattr(STDIN_FILENO, &canmon.tty);
	struct termios raw = canmon.tty;
	raw.c_lflag &= ~(ECHO| ICANON);
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 1;
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
	write(STDOUT_FILENO, "\x1b[?25l", 6);
	/////////////////////////////////////////

	// Buffer initialization.
	// This are displayed before they are populated.
	memset(canmon.last_action, ' ', 32);
	memset(scripts.s_buf, ' ', 4*64);
	canmon.script_idx = 0;

	// Stuff that will fail, probably
	canmon.can_error = init_can(&canmon.can_bus);
	canmon.script_error = init_script();
	
	init_screen();

	char in = '\0';
	while(in != 'q')
	{
		in = '\0';
		handle_input(&in);
		handle_can();
		draw_to_buf();
		render();
		sleep((float)TICK_INTERVAL/1000);
	}
	
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// TUI SCREEN BUFFERING

/*
 * void init_screen()
 *	This function... creates the screen buffer.
 * 	it defaults to 80x40, however will attempt
 * 	to read the terminal size and dynamically allocate
 * 	buffer space to maximize coolness.
 */
void init_screen()
{
	struct winsize tmp;
	if(ioctl(STDOUT_FILENO, TIOCGWINSZ, &tmp) == -1)
	{
		screen.w = 80;
		screen.h = 40;
	}
	else
	{
		screen.w = tmp.ws_col;
		screen.h = tmp.ws_row;
	}
	screen.len = screen.w * screen.h;
	screen.buf = (char*)malloc(screen.len * sizeof(char));
	memset(screen.buf, ' ', screen.len);
}

// This doesn't deserve an explaination, it's just wrapped pointer math
// to determine where char (x,y) is stored in memory.
char* buf_access(int x, int y)
{
	if( x >= screen.w || y >= screen.h) return NULL;
	return &screen.buf[ (y*screen.w) + x ];
}

/*
 * void writeb()
 *	This is like write(), but to a 2D buffer!
 * 	if the supplied string is longer than the
 * 	width of the screen, it does not write.
 */
void writeb(int x, int y, char* c, int len)
{
	if(x+len >= screen.w || y >= screen.h) return;
	strncpy(buf_access(x,y), c, len);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Core program control & functionality stuff

// Just some wrapped escape characters!
void clear() 	{ write(STDOUT_FILENO, "\x1b[2J\x1b[H\x1b[m", 10); }
void newline() 	{ write(STDOUT_FILENO, "\n", 1); }

/*
 * void kill()
 * 	This function cleans up our silly mess.
 * 	When the screen is created, a copy the terminal's settings
 * 	are saved and reapplied when the program exits (here). This
 * 	prevents any non-canonical weirdness from escaping our TUI
 * 	and mucking up the command line. Dynamically allocated memory
 * 	is also freed here.
 */
void kill()
{
	clear();
	write(STDOUT_FILENO, "\x1b[?25h", 6);
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &canmon.tty);
	close(canmon.can_bus);
	free(screen.buf);
}

/*
 * void set_last_action()
 * 	Used to set the action box in the footer of the screen.
 * 	it is hardcoded to a 32 byte length for simplicity.
 */
void set_last_action(char* c)
{
	memset(canmon.last_action, ' ', 32);
	strncpy(canmon.last_action, c, 32);
}

/*
 * void handle_input()
 * 	This function is responsible for reading and interpreting
 * 	user input (one byte at a time). This function also handles
 * 	script selection and execution (as it relies heavily on user
 * 	input). Note that 'q' is handled in main() as to always
 * 	be available in the event of an error.
 */
void handle_input(char* c)
{
	read(STDIN_FILENO, c, 1);

	// Disable input if big-boi program error
	if(canmon.can_error || canmon.script_error) return;
	switch(*c)
	{
		case 'k':	// EMERGENCY STOP
			send_can(&canmon.can_bus, &STOP_FRAME);
			set_last_action(" Sent STOP Frame ");
			break;
		case 'a':	// SEND ALL CLEAR
			send_can(&canmon.can_bus, &AC_FRAME);
			set_last_action(" Sent ALL CLEAR Frame ");
			break;
		case 'm':	// KILL MOTORS
			send_can(&canmon.can_bus, &MOT_FRAME);
			set_last_action(" Set Motor Throttle to ZERO ");	
			break;
		case 's':	// ENTER SCRIPT MODE
			if(canmon.script_mode_en)
				canmon.script_mode_en = 0;
			else
				canmon.script_mode_en = 1;
			set_last_action(" Select a script (SEE HELP) ");
			break;
		case ']':	// DOWN
			if(canmon.script_mode_en && canmon.script_idx < scripts.n-1)
				canmon.script_idx++;
			break;
		case '[':	// UP
			if(canmon.script_mode_en && canmon.script_idx > 0)
				canmon.script_idx--;
			break;		
		case 'r':	// RUN SCRIPT
			if(canmon.script_mode_en)
			{
				if(exec_script() != 0)
				snprintf(canmon.last_action, 32, " !!!FAILED : %s ",
					scripts.scripts[(int)canmon.script_idx].d_name);
				else
				snprintf(canmon.last_action, 32, " Executed: %s ",
					scripts.scripts[(int)canmon.script_idx].d_name);
				canmon.script_mode_en = 0;
			}
			break;
		default: break;
	}	
}

/*
 * void render()
 *	This function copys our character buffer to the screen.
 * 	It also filters out all newlines and NULL terminators to prevent
 * 	output weirdness. Output is copied line by line.
 */
void render()
{
	clear();
	for(int y = 0; y < screen.h; y++)
	{
		for(int x = 0; x < screen.w; x++)
			if(screen.buf[(y*screen.w)+x] == '\0'
				|| screen.buf[(y*screen.w)+x] == '\n')
				screen.buf[(y*screen.w)+x] = ' ';	
		write(STDOUT_FILENO, &screen.buf[(y*screen.w)], screen.w);
		if(y < screen.h-1) newline();
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// String constants, because they have to go somewhere!
char header[] = "MECHATRONICS CAN MONITOR [CLI] v0.0.1 | PRESS 'q' TO QUIT.";
char footer[] = "LATEST ACTION: [                                ] | CURRENT BUS: [      ]";
char can_fail[] = "\x1b[31;7;5m!!! Could not initialize CAN bus !!!\x1b[m";
char script_fail[] = "\x1b[31;7;5m!!! Script initialization has failed !!!\x1b[m";
char info[9][64] =
{
	"These hotkeys are available:",	
	"    K: Inject a kill frame into the selected bus.",	
	"    A: Inject an \'All Clear\' frame into the selected bus.",	
	"    M: Set all motor throttles to ZERO.",
	"    Q: Quit this program and return to the terminal.",
	"    S: Enter \'Script Mode\' to select & run scripts.",
	"    ]: (SCRIPT MODE ONLY) Move selection mark \'*\' down.",
	"    [: (SCRIPT MODE ONLY) Move selection mark \'*\' up.",
	"    R: (SCRIPT MODE ONLY) Execute the selected script.",
};
char ms5837_title[] = "MS5837 Pressure Sensor:"; 
char dvl_title[] = "Doplar Velocity Log (DVL):";
char script_title[] = "LOADED SCRIPTS:";
char script_o_title[] = "CURRENT SCRIPT OUTPUT:";
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// TUI Drawing functions


/*
 * void draw_to_buf()
 *	This function is the 'main' buffered rendering function.
 * 	It is run once per tick, and almost all buffer-writing operations
 * 	occur here. It's basically graphic design at this point!. 
 */
int cur_line = 0;
void draw_to_buf()
{
	// All drawing goes here
	writeb( (screen.w - sizeof(header))/2, 0, header, sizeof(header));			// HEADER
	draw_hline(1);																// HEADER LINE
	draw_hline(screen.h-2);														// FOOTER LINE
	writeb(0,screen.h-1, footer, sizeof(footer));								// FOOTER
	writeb(16, screen.h-1, canmon.last_action, 32);								// FOOTER DATA
	writeb(66, screen.h-1, (char*)CAN_BUS_INTERFACE, 6);						// FOOTER DATA

	// Draw help/info panel
	cur_line = 2;
	for(int i = 0; i < 9; i++)
	{
		writeb(0, cur_line, info[i], sizeof(info[i]));
		cur_line ++;
	}

	// Draw motor sliders and aliases
	draw_hline(cur_line);
	draw_motors(0,cur_line += 1, hw_data.motor_val, 8, 1);
	draw_hline(cur_line += 8);

	// Draw sensor data panels
	draw_ms5837(1,cur_line += 1);
	draw_vline(32,cur_line,6);
	draw_dvl(34, cur_line);
	draw_vline(66, cur_line, 6);
	draw_hline(cur_line += 6);

	// Draw automation panel
	draw_scripts(0,cur_line += 1);
	draw_script_buf(32, cur_line);
	cur_line += scripts.n;

	// Draw error messages (blinky blinky red)
	if(canmon.can_error != 0)
		writeb( (screen.w - sizeof(can_fail)+12)/2,
		screen.h/2, can_fail, sizeof(can_fail));
	if(canmon.script_error != 0)
		writeb( (screen.w - sizeof(script_fail)+12)/2,
		1+(screen.h/2), script_fail, sizeof(script_fail));
}

// More wrapped write calls, just line drawing
void draw_hline(int y) { memset(buf_access(0,y), '_', screen.w); }
void draw_vline(int x, int y, int l)
{
	for( int i = y; i < (y+l); i++)
		writeb( x, i, "|", 1);
}

/*
 * void draw_slider()
 *	This function computes and draws a slider based on a minimum,
 *	maximum, and current value. It scales these values down so
 *	that the slider fits within w width. Due to the limitations
 * 	of integer math, a higher width means greater resolution.
 * 	Note that some widths may cause the slider to break (thus
 * 	w should be a multiple of 5.. believe me)
 */
void draw_slider(int x, int y, int val, int min, int max, int w)
{
	int dif 	= abs(max-min);		// numerical length
	float f 	= (1.0*dif)/w;		// scale factor
	int fval 	= val/f;			// scale to fit w
	int fmin 	= min/f;			// scale to fit w
	int fmax 	= max/f;			// scale to fit w
	int fmid 	= (fmin+fmax)/2;	// scale to fit w

	// allocate slider size
	char* slider = (char*)malloc(w*sizeof(char));
	memset(slider, ' ', w);	

	// limits
	if(fval >= fmax) fval = fmax;
	if(fval <= fmin) fval = fmin;

	// Compute slider position for each char in *slider
	for(int i = 0; i < w; i++)
	{
		int p = fmin+i;
		if(fval >= fmid)
		{
			if( p  >= fmid && p <= fval)
				slider[i] = '|';
		}
		else
		{
			if( p <= fmid && p >= val)
				slider[i] = '|';
		}
		slider[0] = '[';
		slider[w-1] = ']';
	}
	writeb(x, y, slider, w);
	free(slider);	// I want to break free!
}


/*
 * void draw_motors()
 *	This function draws sliders corresponding to motor throttles
 * 	at (x,y). It allows for auto centering of the text block, and
 * 	depending on the program configuration, displays motor aliases.
 */
void draw_motors(int x, int y, int motors[], int len, char center)
{
	char label[] = "MOTOR N: ";
	char val[] = "    ";

	if(y + len >= screen.h) return;
	
	for(int i = 0; i < len; i++)
	{

		// calculate available screen space 
		int space = (screen.w - sizeof(label) - sizeof(val));
		if(SHOW_MOTOR_AILIAS)
			space = (screen.w- sizeof(label) - sizeof(val) - 32);

		// compute slider length in increments of 5.
		// other lengths dont work well.
		int slider_len 	= ( space - space%5 );
		if(center)
			x = (space - slider_len)/2;

		// draw labels, sliders, and values.
		label[6] = i+48;
		sprintf(val, "%4i", motors[i]);
		writeb(x, y+i, label, sizeof(label));
		draw_slider(x + sizeof(label), y+i,
			motors[i], -100, 100, slider_len);
		writeb(x + sizeof(label) + slider_len,
			y+i, val, sizeof(val)-1);

		// display motor alias
		if(SHOW_MOTOR_AILIAS)
		{
			for(int i = 0; i < 8; i++)
			writeb( (x+sizeof(label) + slider_len + sizeof(val) + 1),
				y+i, "|", 1);
			writeb( (x+sizeof(label) + slider_len + sizeof(val) + 4 ),
				y+i, (char*)MOTOR_ALIAS[i], 23);
		}
	}
}

/*
 * void draw_ms5837()
 *	This function formats and draws MS5837 data to the
 *	screen starting at (x,y). it is 3 lines tall.
 */
void draw_ms5837(int x, int y)
{
	char label[32];
	writeb(x, y, ms5837_title, sizeof(ms5837_title));
	snprintf(label, 32,  "    %-20s: %.3f ",
		"Temperature", hw_data.ms5837_temp);
	writeb(x, y + 1, label, 30);
	snprintf(label, 32,  "    %-20s: %.3f ",
		"Depth", hw_data.ms5837_depth);
	writeb(x, y+2, label, 30);
}

/*
 * void draw_dvl()
 *	This function formats and draws DVL data to the
 *	screen starting at (x,y). it is 6 lines tall.
 */
void draw_dvl(int x, int y)
{
	char label[32];
	writeb(x, y, dvl_title, sizeof(dvl_title));
	snprintf(label, 32,  "    %-20s: %.3f ",
		"X Velocity", hw_data.dvl_vel_x);
	writeb(x, y + 1, label, 31);
	snprintf(label, 32,  "    %-20s: %.3f ",
		"Y Velocity", hw_data.dvl_vel_y);
	writeb(x, y + 2, label, 31);
	snprintf(label, 32,  "    %-20s: %.3f ",
		"Z Velocity", hw_data.dvl_vel_z);
	writeb(x, y + 3, label, 31);
	snprintf(label, 32,  "    %-20s: %.3f ",
		"Velocity Error", hw_data.dvl_vel_e);
	writeb(x, y + 4, label, 31);
	snprintf(label, 32,  "    %-20s: %.3f ",
		"Bottom Distance", hw_data.dvl_dist);
	writeb(x, y + 5, label, 31);
	
}

/*
 * void draw_scripts()
 *	This function draws a list of all available scripts.
 *	if 'Script Mode' is enabled, a '*' marks the selected
 * 	script.
 */
void draw_scripts(int x, int y)
{
	char label[32];
	writeb(x,y, script_title, sizeof(script_title));
	for(int i = 0; i < scripts.n; i++)
	{
		snprintf(label, 32, "  > %02i : [%s]",
			i, scripts.scripts[i].d_name);
		writeb(x, y+i+1, label, 31);
		if(canmon.script_mode_en > 0 && i == canmon.script_idx)
			writeb(x,y+i+1, "*", 1);
	}
}

/*
 * void draw_script_buf()
 * This function copies the contents of the script output buffer
 * to the main screen buffer starting at (x,y). This function
 * allows us to easily shift the location of the output panel
 * depending on screen size.
 */
void draw_script_buf(int x, int y)
{
	writeb(x,y, script_o_title, sizeof(script_o_title));
	for(int i = 0; i < 4; i++)
	{
		writeb(x, y+i+1, "~", 1);
		writeb(x+2, y+i+1, scripts.s_buf[i],
			sizeof(scripts.s_buf[i]));
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Ugly CAN frame decoding

/*
 * void _handle_dvl()
 *	Helper function. see explaination below.
 */
void _handle_dvl(struct frame_data* data)
{
	switch(data->topic)
	{
		case 0x0000:	// INFO
			break;
		case 0x0002:	// VEL X
			memcpy(&hw_data.dvl_vel_x, &data->data, sizeof(u_int32_t));
			break;
		case 0x0003:	// VEL Y
			memcpy(&hw_data.dvl_vel_y, &data->data, sizeof(u_int32_t));
			break;
		case 0x0004:	// VEL Z
			memcpy(&hw_data.dvl_vel_z, &data->data, sizeof(u_int32_t));
			break;
		case 0x0005:	// VEL E
			memcpy(&hw_data.dvl_vel_e, &data->data, sizeof(u_int32_t));
			break;
		case 0x000A:	// DIST
			memcpy(&hw_data.dvl_dist, &data->data, sizeof(u_int32_t));
			break;
	}
}

/*
 * void _handle_ms5837()
 *	Helper function. see explaination below.
 */
void _handle_ms5837(struct frame_data* data)
{
	switch(data->topic)
	{
		case 0x0000:	// INFO
			break;
		case 0x0002:	// DEPTH
			memcpy(&hw_data.ms5837_depth, &data->data, sizeof(u_int32_t));
			break;
		case 0x0003:	// TEMP
			memcpy(&hw_data.ms5837_temp, &data->data, sizeof(u_int32_t));
			break;
	}	
}

/*
 * void handle_can()
 *	This is a very not elegant way of decoding CAN frames, however it
 *	works damnit! Frames are sorted (motor or data?) and then decoded
 *	depending on what data they contain. Device data frames (DRES) are
 *	copied into a formatted frame_data struct and passed to helper
 * 	functions (SEE ABOVE) to sort data into the correct variable.
 */
void handle_can()
{
	struct frame_data data;
	memset(&data, 0, sizeof(data));
	if(recv_can(&canmon.can_bus, &canmon.frame) != 0)
		return;
	switch(canmon.frame.can_id)
	{
		case 0x010:	// MOTOR
			for(int i = 0; i < canmon.frame.can_dlc; i++)
				hw_data.motor_val[i] = (int8_t)canmon.frame.data[i];
			break;
		case 0x021: // DRES
			if(canmon.frame.can_dlc != 8) return;
			memcpy(&data.dev, canmon.frame.data, sizeof(u_int16_t));
			memcpy(&data.topic, canmon.frame.data + sizeof(u_int16_t),
				sizeof(u_int16_t));
			memcpy(&data.data, canmon.frame.data + sizeof(u_int32_t),
				sizeof(u_int32_t));
			switch(data.dev)
			{
				case ID_WAYFDVL:
					_handle_dvl(&data);
					break;
				case ID_MS5837:
					_handle_ms5837(&data);
					break;
				default:
					break;
			}		
			break;
		default:
	}
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// CAN Implementation

/*
 * init_can():
 * 	
 * 	Creates a new socket (and assigns it to pointer passed to function)
 * 	and binds it to the "vcan0" virtual CAN bus. this is how the program
 * 	will interact with the rest of the "robot".
 *
 * RETURN VALUE:
 * 	This function should return 0 if successful. A non-zero return value
 *  would most likelya indicate that one or more of the initialization steps
 * 	did not complete successfully.
 */
int init_can(int* can_sock)
{
	int _ret = 0;
	struct can_filter _filter = {0x000, 0x000};
	struct sockaddr_can _can_addr;
	struct ifreq ifr;

	strncpy(ifr.ifr_name, CAN_BUS_INTERFACE, sizeof(ifr.ifr_name));
		
	*can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW); 
	int flags = fcntl(*can_sock, F_GETFL);
	_ret += fcntl(*can_sock, F_SETFL, flags | O_NONBLOCK);
	_ret += ioctl(*can_sock, SIOCGIFINDEX, &ifr); 
	_can_addr.can_family = AF_CAN;
	_can_addr.can_ifindex = ifr.ifr_ifindex;
	_ret += bind(*can_sock, (struct sockaddr*)&_can_addr,
		sizeof(&_can_addr));
	_ret += setsockopt(*can_sock, SOL_CAN_RAW,
		CAN_RAW_FILTER, &_filter, sizeof(&_filter));

	return _ret;
}

/*
 * send_can():
 * 	
 * 	Sends a CAN frame through the specified socket. If bytes are
 * 	written, the function returns 0. If unsuccessful, the function
 * 	returns -1.
 */
int send_can(int* can_sock, struct can_frame* frame)
{
	if( write(*can_sock, frame, sizeof(struct can_frame)) > 0)
		return 0;
	return -1;
}

/*
 * recv_can():
 * 	
 * 	Receives a CAN frame from the specified socket. If bytes are
 * 	received, the function returns 0. If unsuccessful, the function
 * 	returns -1.
 */
int recv_can(int* can_sock, struct can_frame* frame)
{
	if( read(*can_sock, frame, sizeof(struct can_frame)) > 0)
		return 0;
	return -1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Automation

/*
 * int init_script()
 *	This function attempts to index all scripts avaiable to the
 *	program (SEE CONFIG). If successful, a dynamically allocated
 * 	array of 'files' will be generated with the name of all scripts.
 * 	This function will return -1 if unsucessful, and 0 otherwise. 
 */
int init_script()
{
	memset(&scripts, 0, sizeof(scripts));
	memset(scripts.dir, ' ', sizeof(scripts.dir));
	char* _home = getenv("HOME");	// This may not work on all systems.. but eh
	if(_home == NULL) return -1;
	snprintf(scripts.dir, sizeof(scripts.dir),
		"%s/%s/", _home, SCRIPT_DIR);	

	scripts.d_fd = opendir(scripts.dir);

	if(!scripts.d_fd) return -1;

	// Read the directory and count number of scripts
	struct dirent* tmp;
	while( (tmp = readdir(scripts.d_fd)) != NULL)
	{
		if(strstr(tmp->d_name, SCRIPT_EXT) != NULL)
			scripts.n += 1;
	}

	// Allocate memory for our script names 
	scripts.scripts = (struct dirent*)calloc(scripts.n,
		sizeof(struct dirent));
	rewinddir(scripts.d_fd);

	// Read script names into our freshly allocated memor
	int i = 0;
	while( (tmp = readdir(scripts.d_fd)) != NULL)
	{
		if(strstr(tmp->d_name, SCRIPT_EXT) != NULL)
		{
			scripts.scripts[i] = *tmp;
			i += 1;
		}
	}
	
	closedir(scripts.d_fd);
	
	return 0;
}

/*
 * int exec_script()
 *	This function attempts to run the selected script with
 *	Bash. Script output is read as a stream, and only the first
 * 	4 lines are copied to the script output buffer (so only make
 * 	scripts that output <= 4 lines of text!) If the script fails
 * 	to open or execute, the function returns a non-zero value.
 * 	The function should return 0 if successful.
 */
int exec_script()
{
	snprintf(scripts.s_buf[0], 64, "/bin/sh -e %s/%s/%s",
		getenv("HOME"), SCRIPT_DIR,
		scripts.scripts[(int)canmon.script_idx].d_name);	
	scripts.s_fd = popen(scripts.s_buf[0], "r");
	if(scripts.s_fd == NULL) return -1;
	memset(scripts.s_buf, ' ', sizeof(scripts.s_buf));
	for(int i = 0; i < 4; i++)
	{
		if( fgets(scripts.s_buf[i], sizeof(scripts.s_buf[i]),
			scripts.s_fd) == NULL)
			break;
	}
	pclose(scripts.s_fd);
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
TODO:
	- Implement embedded status polling?
	- Implement bash scripting system?
	- Implement CAN state tester
*/
