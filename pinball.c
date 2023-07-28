#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <allegro.h>

#include <math.h>
#include <stdlib.h>
#include "ptask.h"
#define MAX_BALLS			20		
#define SCALEFACTOR_TIME	8
#define SCREEN_COLOR		198
#define COMMAND_COLOR		0
#define SCREEN_WIDTH		1024
#define SCREEN_HEIGHT		768
#define LINE_TICKNESS		2
#define LINE_COLOR			9 
#define LETTER_COLOR		11
#define X1					XBOX + 100
#define X2					180
#define X3					50
#define Y1					200													//posizione verticale casella comandi
#define Y2					Y1/4												//spaziamento fra una casella di testo e l altra
#define XBOX				450													//dimensioni spazio in cui rimbalza la pallina
#define YBOX				767
#define XB					95													//dimensioni pavimento
#define TETA				M_PI/4												//angolo pavimento [rad]
#define YINF				150
#define YSUP				( YINF + XB*tan(TETA) )								//altezza piano inclinato
#define YF					( YINF + PADDLE_LENGHT*sin(LPADDLE_DOWN) - 5 )		//posizione di fine gioco							
#define POLYGON_COLOR		3		

#define BALL_RADIUS			10
#define BALL_COLOR_I		15													//colore iniziale pallina

#define GRAVITY				9.8													//accelerazione di gravita' [m/s2]
#define DAMPER				0.9
#define AMPL				1.1
#define SPEED_LIMIT			130													//limite di velocità
#define VDX					5													// decremento di velocità lungo x
#define VDY					5													//decremento di velocità lungo y					

#define XI					300													//condizioni iniziali da cui far partire la pallina			
#define YI					700
#define VXI					-25													//[m/s]

#define PADDLE_LENGHT		110
#define PADDLE_TICKNESS		30
#define PADDLE_TIP_RADIUS	5
#define PADDLE_VEL			0.8													//[rad/s]
#define PADDLE_COLOR		51

#define LPADDLE_DOWN		-M_PI/6												//inclinazione paddle sinistro a riposo
#define LPADDLE_UP			M_PI/9												//inclinazione massima paddle sinistro alzato

#define RPADDLE_DOWN		M_PI/6												//inclinazione paddle destro a riposo
#define RPADDLE_UP			-M_PI/9												//inclinazione massima paddle destro alzato

#define XR1					XBOX/2												//posizioni repulsori circolari
#define YR1					500
#define R1_RADIUS			35

#define XR2					XBOX/3
#define YR2					600
#define R2_RADIUS			35

#define REPULSORS_COLOR		11

#define XS					100													//posizioni repulsori lineari
#define YS					350
#define LS					70													//lunghezza repulsore lineare
#define TS					20													//spessore repulsore lineare
#define TETA_S				M_PI/6
#define S_COLOR				11 

//task_b Parameters----BALL-------
#define taskB_index			0             
#define taskB_period		15
#define taskB_rdline		15
#define taskB_prio			30

//task_p Parameters----PADDLE-------
#define taskP_index			MAX_BALLS 
#define taskP_period		20
#define taskP_rdline		20
#define taskP_prio			25


//task_g Parameters----GRAPHICS-------
#define taskG_index			MAX_BALLS + 1 
#define taskG_period		20
#define taskG_rdline		20
#define taskG_prio			20


//task_t Parameters----TASTIERA-------
#define taskT_index			MAX_BALLS + 2                
#define taskT_period		120
#define taskT_rdline		120
#define taskT_prio			15
//Dichiarazione variabili globali---------------------------------------------------------------------------------------------------------------------------------------------------------

struct ball_status {                          
	float	x;		
	float	y;
	float	vx;
	float	vy;
	float	r;							//raggio pallina
	int	bc;								//bounce counter
	int	color;							//ball color
}; 
struct paddle_status {                          
	float xp;							//posizione lungo x fulcro paddle
	float yp;							//posizione lungo y fulcro paddle
	float tetap;						//angolo di inclinazione paddle
	float omegap;						//velocita' angolare paddle
	float l;							//lunghezza paddle
	float t;							//spessore paddle
}; 
struct ball_status bs[MAX_BALLS];
struct paddle_status lps;				//stato paddle sinistro(left)
struct paddle_status rps;				//stato paddle destro(right)

int nb = 0;								//numero palline in gioco  
//-------------------------------------------------------------------------------------------------------------------
//DICHIARAZIONE FUNZIONI AUSILIARIE DA UTILIZZARE NEI TASK-----------------------------------------------------------

void write_command(BITMAP* buf);																
void screen_init(void);																	
void ball_init(int i);																	
float dist_pl(float xp, float yp, float teta, float q); 								
float dist_pr(float xp, float yp, float xr, float yr);									
void coll_point(float xp, float yp, float teta, float q, int i); 						
void coll_point_pr(float xp, float yp, float rp, float xr, float yr, float rr, int i);	
void coll_point_bb(int i, int k);														
void bounce_vel(float teta, int i); 													
void bounce_vel_lpaddle(float teta, int i);												
void bounce_vel_rpaddle(float teta, int i);												
void bounce_vel_r(float xr, float yr, float rr, int i); 								
void bounce_vel_bb(int i, int k);														
void handle_bounce(float w, float h, int i); 											
void handle_bounce_ulr(float w, float h, int i);										
void handle_bounce_floor(int i);														
void handle_bounce_paddle(int i);													
void handle_bounce_r(float xr, float yr, float r, int i);								
void bounce_s(int i, float teta1, float q1, float teta2, float q2, float teta3, float q3, float teta4, float q4);	
void handle_bounce_s(int i);															
void handle_bounce_bb(int i);
float ball_speed(int i);																
void speed_limiter(int i);																

void paddle_init(void); 									
void key_paddle(float dt);

void draw_bck(BITMAP* buf);										
void draw_ball(BITMAP* buf);										
void draw_lpaddle(BITMAP* buf);									
void draw_rpaddle(BITMAP* buf);									
void write_dmiss(BITMAP* buf);										
void write_points(BITMAP* buf);									
void write_max_speed(BITMAP* buf);									
void write_remaining_balls(BITMAP* buf);							
void draw_repulsors(BITMAP* buf);									
void draw_s(BITMAP* buf);											
											
void key_T(void);													
//-----------------------------------------------------------------------------------------------------------------------
//DICHIARAZIONI FUNZIONI DEI TASK----------------------------------------------------------------------------------------

void* taskB(void* p);	//ball kynematics                     
void* taskP(void* p);	//paddle kynematics
void* taskG(void* p);	//graphics 
void* taskT(void* p);	//tastiera
//----------------------------------------------------------------------------------------------------------------------
//DEFINIZIONE FUNZIONI AUSILIARE DA UTILIZZARE NEI TASK-----------------------------------------------------------------

//-----------------------------------------------------------------------------------
// La funzione write_command scrive i comandi di gioco su screen
//-----------------------------------------------------------------------------------
void write_command(BITMAP* buf)
{
	textout_ex(buf, font, "PRESS SPACE TO ADD BALLS", X1, Y2, LETTER_COLOR, -1);
	textout_ex(buf, font, "PRESS ESC TO QUIT THE GAME", X1, 2*Y2, LETTER_COLOR, -1);
	textout_ex(buf, font,"SELECT BALL COLOR", X1, 3*Y2, LETTER_COLOR, -1);
	textout_ex(buf, font, "V", X1 + X2, 3*Y2, 10, -1);
	textout_ex(buf, font, "B", X1 + X2 + X3, 3*Y2, 15, -1);
	textout_ex(buf, font, "R", X1 + X2 + 2*X3, 3*Y2, 12, -1);
	textout_ex(buf, font, "G", X1 + X2 + 3*X3, 3*Y2, 14, -1);
}

//-----------------------------------------------------------------------------------
// La funzione screen_init inizializza lo screen
//-----------------------------------------------------------------------------------
void screen_init(void)
{
	int i;
	allegro_init();             
	install_keyboard();
	set_color_depth(8);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0);
	clear_to_color(screen, COMMAND_COLOR);
	
	paddle_init();
	for( i = 0; i < MAX_BALLS; i++) ball_init(i);
}

//-----------------------------------------------------------------------------------
// La funzione ball_init inizializza stato palla
//-----------------------------------------------------------------------------------
void ball_init(int i)
{
	bs[i].x = XI;
	bs[i].y = YI;
	bs[i].vx = 0;
	bs[i].vy = 0;
	bs[i].r = BALL_RADIUS;
	bs[i].bc = 0;
	bs[i].color = BALL_COLOR_I; //inizializzo la pallina con colore verde
}

//-----------------------------------------------------------------------------------
// La funzione dist_pl calcola la distanza fra un punto ed una retta 
// con eq retta espressa in forma esplicita
//-----------------------------------------------------------------------------------
float dist_pl(float xp, float yp, float teta, float q)
{
	float m = tan(teta);
	return fabs(yp - (m*xp + q))/sqrt(1 + m*m);	
}

//-----------------------------------------------------------------------------------
// La funzione dist_pr calcola la distanza fra il centro della pallina 
// e il centro di un repulsore 
//-----------------------------------------------------------------------------------
float dist_pr(float xp, float yp, float xr, float yr)
{
	return sqrt( (xr - xp) * (xr - xp) + (yr - yp) * (yr - yp));
}

//-----------------------------------------------------------------------------------
// La funzione coll_point calcola il punto di collisione fra pallina e linea 
// e da valore corretto a coordinate pallina
//-----------------------------------------------------------------------------------
void coll_point(float xp, float yp, float teta, float q, int i)
{
	float m = tan(teta);												//coefficiente angolare
	float xh = (bs[i].y + bs[i].x/tan(teta) - q)/(m + 1/tan(teta));		//punto lungo x da cui passa la retta
	float yh = m*xh +q;													//punto lungo y da cui passa la retta
	bs[i].x = xh - bs[i].r*sin(teta);									//calcolo centro della pallina corretto post collisione
	bs[i].y = yh + bs[i].r*cos(teta);  
}

//-----------------------------------------------------------------------------------
// La funzione coll_point_pr calcola il punto di collisione fra pallina e repulsore 
// e da valore corretto a coordinate pallina
//-----------------------------------------------------------------------------------
void coll_point_pr(float xp, float yp, float rp, float xr, float yr, float rr, int i)
{
	float c, d, nx, ny;
	d = dist_pr(xp, yp, xr, yr);
	c = (rp + rr) - d ;
	nx = (xr - xp)/dist_pr(xp, yp, xr, yr);
	ny = (yr - yp)/dist_pr(xp, yp, xr, yr);
	bs[i].x -= c*nx;
	bs[i].y -= c*ny;
}

//-----------------------------------------------------------------------------------
// La funzione coll_point_bb calcola il punto di collisione fra due palline 
// e da valori corretti a coordinate palline
//-----------------------------------------------------------------------------------
void coll_point_bb(int i, int k)
{
	float c, d, nx, ny;
	d = dist_pr(bs[i].x, bs[i].y, bs[k].x, bs[k].y);
	c = ((bs[i].r + bs[k].r) - d)/2;
	nx = (bs[k].x - bs[i].x)/d;
	ny = (bs[k].y - bs[i].y)/d;
	bs[i].x -= c*nx;
	bs[i].y -= c*ny;
	bs[k].x += c*nx;
	bs[k].y += c*ny;
}

//-----------------------------------------------------------------------------------
// La funzione bounce_vel calcola la velocita' della pallina dopo la collisione
// fra essa ed uno dei due piani inclinati
//-----------------------------------------------------------------------------------
void bounce_vel(float teta, int i)
{
	float vn, vt;
	
	vn =  DAMPER*( bs[i].vx*sin(teta) + fabs(bs[i].vy)*cos(teta) );		//velocita' normale a piano su cui avviene collisione
	vt = bs[i].vx*cos(teta) - fabs(bs[i].vy)*sin(teta);					//velocita' tangenziale a piano su cui avviene collisione
	
	bs[i].vx = vt*cos(teta) - fabs(vn)*sin(teta);						//velocita' palla post collisione
	bs[i].vy = vt*sin(teta) + fabs(vn)*cos(teta);
}

//-----------------------------------------------------------------------------------
// La funzione bounce_vel_lpaddle calcola la velocita' della pallina dopo la collisione
// fra essa ed il paddle sinistro
//-----------------------------------------------------------------------------------
void bounce_vel_lpaddle(float teta, int i)
{
	float vn, vt;
	
	vn =  DAMPER*( bs[i].vx*sin(teta) + fabs(bs[i].vy)*cos(teta) ) + 20*lps.omegap;
	vt = bs[i].vx*cos(teta) - fabs(bs[i].vy)*sin(teta);
	
	bs[i].vx = vt*cos(teta) - fabs(vn)*sin(teta);
	bs[i].vy = vt*sin(teta) + fabs(vn)*cos(teta);	
}

//-----------------------------------------------------------------------------------
// La funzione bounce_vel_rpaddle calcola la velocita' della pallina dopo la collisione
// fra essa ed il paddle destro
//-----------------------------------------------------------------------------------
void bounce_vel_rpaddle(float teta, int i)
{
	float vn, vt;
	
	vn =  DAMPER*( bs[i].vx*sin(teta) + fabs(bs[i].vy)*cos(teta) ) + 20*rps.omegap;
	vt = bs[i].vx*cos(teta) - fabs(bs[i].vy)*sin(teta);
	bs[i].vx = vt*cos(teta) - fabs(vn)*sin(teta);
	bs[i].vy = vt*sin(teta) + fabs(vn)*cos(teta);
}

//-----------------------------------------------------------------------------------
// La funzione bounce_vel_r calcola la velocita' della pallina dopo la collisione
// fra essa ed un repulsore circolare
//-----------------------------------------------------------------------------------
void bounce_vel_r(float xr, float yr, float rr, int i)
{
	float d, nx, ny, tx, ty, vn, vt;
	
	d = dist_pr(bs[i].x, bs[i].y, xr, yr);
	nx = (xr - bs[i].x)/d;
	ny = (yr - bs[i].y)/d;
	tx = -ny;
	ty = nx;
	vn = AMPL*(bs[i].vx*nx + bs[i].vy*ny);
	vt = bs[i].vx*tx + bs[i].vy*ty;
	bs[i].vx = -vn*nx +vt*tx;
	bs[i].vy = -vn*ny +vt*ty;
}

//-----------------------------------------------------------------------------------
// La funzione bounce_vel_bb calcola la velocita' della pallina dopo la collisione
// fra essa ed un altra pallina
//-----------------------------------------------------------------------------------
void bounce_vel_bb(int i, int k)
{
	float d, nx, ny, tx, ty, vn1, vt1, vn2, vt2;
	
	d = dist_pr(bs[i].x, bs[i].y, bs[k].x, bs[k].y);
	nx = (bs[k].x - bs[i].x)/d;
	ny = (bs[k].y - bs[i].y)/d;
	tx = -ny;
	ty = nx;
	vn1 = (bs[i].vx*nx + bs[i].vy*ny);
	vt1 = bs[i].vx*tx + bs[i].vy* ty;
	vn2 = (bs[k].vx*nx + bs[k].vy*ny);
	vt2 = bs[k].vx*tx + bs[k].vy* ty;
	bs[i].vx = vn2*nx + vt1*tx;
	bs[i].vy = vn2*ny + vt1*ty;
	bs[k].vx = vn1*nx + vt2*tx;
	bs[k].vy = vn1*ny + vt2*ty;
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce gestisce le collisioni della pallina 
//-----------------------------------------------------------------------------------
void handle_bounce(float w, float h, int i)
{
	handle_bounce_ulr(w, h, i);
	handle_bounce_floor(i);
	handle_bounce_paddle(i);
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce_ulr gestisce  gli urti fra
// soffitto parete sinistra e destra (up left right)
//-----------------------------------------------------------------------------------
void handle_bounce_ulr(float w, float h, int i) 
{
	if ( ( bs[i].x - ( bs[i].r ) ) <= 0 ) {					//urto parete sinistra
			
			bs[i].x = bs[i].r;
			bs[i].vx = -bs[i].vx;
	}
    
	if ( ( bs[i].x + ( bs[i].r ) ) >= w ) {					//urto parete destra
    		
			bs[i].x = w - bs[i].r;
			bs[i].vx = -bs[i].vx;
	}
	
	if ( ( bs[i].y + ( bs[i].r )) >= h ) {					//urto soffitto
    		
			bs[i].y = h - bs[i].r;
			bs[i].vy = -DAMPER*bs[i].vy;
	}
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce_floor gestisce urti con pavimento 
//-----------------------------------------------------------------------------------
void handle_bounce_floor(int i)					
{
	float teta1 = -(TETA);
	float q1 = YSUP;
	float teta2 = TETA;
	float q2 = YINF - tan(teta2)*(XBOX - XB);
	
	if ( (dist_pl(bs[i].x, bs[i].y, teta1, q1) <= bs[i].r) && ((bs[i].x - bs[i].r) <= XB)){
		
		coll_point(bs[i].x, bs[i].y, teta1, q1, i);
		bounce_vel( teta1, i );
	}
	
	if ( (dist_pl(bs[i].x, bs[i].y, teta2, q2) <= bs[i].r) && ((bs[i].x + bs[i].r)>= (XBOX - XB))){
		
		coll_point(bs[i].x, bs[i].y, teta2, q2, i);
		bounce_vel( teta2, i );
	}
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce_paddle gestisce urti con paddle
//-----------------------------------------------------------------------------------
void handle_bounce_paddle(int i)				
{
	float q3 = lps.yp - tan(lps.tetap)*lps.xp;
	float q4 = rps.yp - tan(rps.tetap)*rps.xp;

	if ( ( dist_pl(bs[i].x, bs[i].y, lps.tetap, q3) <= bs[i].r) && ( (bs[i].x - bs[i].r) > XB ) && ( (bs[i].x - bs[i].r) <= (XB + lps.l*cos(lps.tetap)) )){
		
		coll_point(bs[i].x, bs[i].y, lps.tetap, q3, i);
		bounce_vel_lpaddle(lps.tetap, i);
	}
	
	if ( (dist_pl(bs[i].x, bs[i].y, rps.tetap, q4) <= bs[i].r) && ( (bs[i].x + bs[i].r) >= (XBOX - (XB + rps.l*cos(rps.tetap) ) ) ) && ( (bs[i].x + bs[i].r ) < (XBOX - XB) ) ) {
		
		coll_point(bs[i].x, bs[i].y, rps.tetap, q4, i);
		bounce_vel_rpaddle(rps.tetap, i);
	}
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce_r gestisce urti con repulsore circolare
// di centro (xr, yr) e raggio r
//-----------------------------------------------------------------------------------
void handle_bounce_r(float xr, float yr, float r, int i)		
{
	
	if ( dist_pr(bs[i].x, bs[i].y, xr, yr) <= (bs[i].r + r) ){
		
		coll_point_pr(bs[i].x, bs[i].y, bs[i].r, xr, yr, r, i);
		bounce_vel_r(xr, yr, r, i);
		bs[i].bc++;
	}
}

//-----------------------------------------------------------------------------------
// La funzione bounce_s verifica se si hanno le condizioni di urto con
// repulsori lineari 
//-----------------------------------------------------------------------------------
void bounce_s(int i, float teta1, float q1, float teta2, float q2, float teta3, float q3, float teta4, float q4)
{
	if ( (dist_pl(bs[i].x, bs[i].y, teta1, q1) <= bs[i].r) && ((bs[i].x - bs[i].r) > XS) && ((bs[i].x + bs[i].r) < (XS + LS*cos(TETA_S)))){
		
		coll_point(bs[i].x, bs[i].y, teta1, q1, i);
		bounce_vel( teta1, i );
		bs[i].bc++;
	}

	if ( (dist_pl(bs[i].x, bs[i].y, teta2, q2) <= bs[i].r) && ((bs[i].x - bs[i].r) > XS) && ((bs[i].x + bs[i].r) < (XS + LS*cos(TETA_S)))){
		
		coll_point(bs[i].x, bs[i].y, teta2, q2, i);
		bounce_vel( teta2, i );
		bs[i].bc++;
	}

	if ( (dist_pl(bs[i].x, bs[i].y, teta3, q3) <= bs[i].r) && ( (bs[i].x + bs[i].r) < (XBOX - XS) ) && ((bs[i].x - bs[i].r) > (XBOX - XS- LS*cos(TETA_S)))){
		
		coll_point(bs[i].x, bs[i].y, teta3, q3, i);
		bounce_vel( teta3, i );
		bs[i].bc++;
	}

	if ( (dist_pl(bs[i].x, bs[i].y, teta4, q4) <= bs[i].r) && ( (bs[i].x + bs[i].r) < (XBOX - XS) ) && ((bs[i].x - bs[i].r) > (XBOX - XS - LS*cos(TETA_S)))){
		
		coll_point(bs[i].x, bs[i].y, teta4, q4, i);
		bounce_vel( teta4, i );
		bs[i].bc++;
	}
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce_s gestisce urti con repulsori lineari
//-----------------------------------------------------------------------------------
void handle_bounce_s(int i)						
{
	float teta1 = (TETA_S - M_PI);
	float q1 = YS -tan(teta1)*XS;
	float teta2 = TETA_S;
	float q2 = YS + TS - tan(teta2)*(XS);
	float teta3 = (M_PI - TETA_S);
	float q3 = YS - tan(teta3)*(XBOX - XS);
	float teta4 = -TETA_S;
	float q4 = YS + TS - tan(teta4)*(XBOX - XS);

	bounce_s(i, teta1, q1, teta2, q2, teta3, q3, teta4, q4);
	handle_bounce_r(XS, YS + TS/2, TS/2, i);
	handle_bounce_r(XS + LS*cos(TETA_S), YS + LS*sin(TETA_S) + TS/2, TS/2, i);
	handle_bounce_r(XBOX - XS, YS + TS/2, TS/2, i);
	handle_bounce_r( XBOX - XS - LS*cos(TETA_S), YS + LS*sin(TETA_S) + TS/2, TS/2, i);
}

//-----------------------------------------------------------------------------------
// La funzione handle_bounce_bb gestisce urti fra pallina i-esima e
// tutte le altre palline in gioco
//-----------------------------------------------------------------------------------
void handle_bounce_bb(int i)
{
	int k;
	float d;
	
	for (k = 0; k < nb; k++){
		
		if ( (dist_pr(bs[i].x, bs[i].y, bs[k].x, bs[k].y) <= (bs[i].r + bs[k].r)) && (k != i) && (bs[k].y > YF) ){
			
			coll_point_bb(i, k);
			bounce_vel_bb(i, k);
		}
	}	
}

//-----------------------------------------------------------------------------------
// La funzione ball_speed fornisce la velocita' scalare di una pallina
//-----------------------------------------------------------------------------------
float ball_speed(int i)
{
	return sqrt( bs[i].vx * bs[i].vx + bs[i].vy * bs[i].vy );
}

//-----------------------------------------------------------------------------------
// La funzione speed_limiter limita velocita' delle palline
//-----------------------------------------------------------------------------------
void speed_limiter(int i)
{
	if ( (ball_speed(i) >= SPEED_LIMIT) && (bs[i].vx > 0) && (bs[i].vy > 0) ){
		
		bs[i].vx -= VDX;
	}

	if ( (ball_speed(i) >= SPEED_LIMIT) && (bs[i].vx > 0) && (bs[i].vy < 0) ){
		
		bs[i].vx -= VDX;
		bs[i].vy += VDY;
	}

	if ( (ball_speed(i) >= SPEED_LIMIT) && (bs[i].vx < 0) && (bs[i].vy > 0) ){
		
		bs[i].vx += VDX;
	}

	if ( (ball_speed(i) >= SPEED_LIMIT) && (bs[i].vx < 0) && (bs[i].vy < 0) ){
		
		bs[i].vx += VDX;
		bs[i].vy += VDY;
	}
}

//-----------------------------------------------------------------------------------
// La funzione paddle_init inizializza gli stati dei due paddle
//-----------------------------------------------------------------------------------
void paddle_init(void)
{
	lps.xp = XB;
	lps.yp = YINF;
	lps.tetap = LPADDLE_DOWN;
	lps.omegap = 0;
	lps.l = PADDLE_LENGHT;
	lps.t = PADDLE_TICKNESS;
	rps.xp = XBOX - XB;
	rps.yp = YINF;
	rps.tetap = RPADDLE_DOWN;
	rps.omegap = 0;
	rps.l = PADDLE_LENGHT;
	rps.t = PADDLE_TICKNESS;
}

//-----------------------------------------------------------------------------------
// La funzione draw_bck disegna lo sfondo
//-----------------------------------------------------------------------------------
void draw_bck(BITMAP* buf)    
{
	int pointsL[8] = {0, SCREEN_HEIGHT - YSUP + 5 , 0, SCREEN_HEIGHT - 1 - YSUP, XB, SCREEN_HEIGHT - 1 - YINF, XB, SCREEN_HEIGHT - YINF + 5};
	int pointsR[8] = {XBOX - XB, SCREEN_HEIGHT - YINF + 5, XBOX - XB, SCREEN_HEIGHT - 1 - YINF, XBOX, SCREEN_HEIGHT - 1 - YSUP, XBOX, SCREEN_HEIGHT - YSUP + 5};
	
	rectfill(buf, 0, SCREEN_HEIGHT - 1, XBOX, SCREEN_HEIGHT - 1 - YBOX, SCREEN_COLOR);
	polygon(buf, 4, pointsL, POLYGON_COLOR);
	polygon(buf, 4, pointsR, POLYGON_COLOR);
	rectfill(buf, XBOX + LINE_TICKNESS + 1 , SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1, Y1 + LINE_TICKNESS + 1, COMMAND_COLOR);
	draw_repulsors(buf);
	draw_s(buf);
	rectfill(buf, XBOX, 0, XBOX + LINE_TICKNESS, SCREEN_HEIGHT -1, LINE_COLOR);		//lineee di delimitazione
	rectfill(buf, XBOX, Y1, SCREEN_WIDTH - 1, Y1 + LINE_TICKNESS, LINE_COLOR);		//linee di delimitazione
	write_command(buf);

	if ( nb == 0){

		circlefill(buf, X1 + X2 + 4*X3, 3*Y2, BALL_RADIUS + 1, COMMAND_COLOR );
		circlefill(buf, X1 + X2 + 4*X3, 3*Y2, BALL_RADIUS, bs[0].color );
	}
	else
		circlefill(buf, X1 + X2 + 4*X3, 3*Y2, BALL_RADIUS + 1, COMMAND_COLOR );
}

//-----------------------------------------------------------------------------------
// La funzione draw_ball disegna tutte le palline attualmente in gioco
//-----------------------------------------------------------------------------------
void draw_ball(BITMAP* buf)
{
	int i;
	int xg, yg;
	
	for (i = 0; i < nb; i++ ){
		
		xg = bs[i].x;
		yg =  (SCREEN_HEIGHT-1) - bs[i].y;
		circlefill(buf, xg, yg, bs[i].r, bs[i].color);
	}
}

//-----------------------------------------------------------------------------------
// La funzione draw_lpaddle disegna il paddle sinistro
//-----------------------------------------------------------------------------------
void draw_lpaddle(BITMAP* buf)
{
	int xp = lps.xp;
	int yp = (SCREEN_HEIGHT - 1) - lps.yp;
	int x1 = lps.xp + lps.t*sin(lps.tetap);
	int y1 = (SCREEN_HEIGHT - 1) - (lps.yp - lps.t*cos(lps.tetap));
	int x2 = lps.xp + lps.l*cos(lps.tetap);
	int y2 = (SCREEN_HEIGHT - 1) - (lps.yp + lps.l*sin(lps.tetap));
	int x3 = lps.xp + lps.l*cos(lps.tetap) + 2*PADDLE_TIP_RADIUS*sin(lps.tetap);
	int y3 = (SCREEN_HEIGHT - 1) - (lps.yp + lps.l*sin(lps.tetap) - 2*PADDLE_TIP_RADIUS*cos(lps.tetap));
	int xc1 = lps.xp + lps.l*cos(lps.tetap) + PADDLE_TIP_RADIUS*sin(lps.tetap);
	int yc1 = (SCREEN_HEIGHT - 1) - (lps.yp + lps.l*sin(lps.tetap) - PADDLE_TIP_RADIUS*cos(lps.tetap));
	int xc2 = lps.xp + 0.5*lps.t*sin(lps.tetap);
	int yc2 = (SCREEN_HEIGHT - 1) - (lps.yp - 0.5*lps.t*cos(lps.tetap));
	int points[8] = {xp, yp, x1, y1, x3, y3, x2, y2};
	
	polygon(buf, 4,points, PADDLE_COLOR);
	circlefill(buf, xc1, yc1, PADDLE_TIP_RADIUS - 1, PADDLE_COLOR);
	circlefill(buf, xc2, yc2, 0.5*lps.t, PADDLE_COLOR);
}

//-----------------------------------------------------------------------------------
// La funzione draw_rpaddle disegna il paddle destro
//-----------------------------------------------------------------------------------
void draw_rpaddle(BITMAP* buf)
{
	int xp = rps.xp;
	int yp = (SCREEN_HEIGHT - 1) - rps.yp;
	int x1 = rps.xp + rps.t*sin(rps.tetap);
	int y1 = (SCREEN_HEIGHT - 1) - (rps.yp - rps.t*cos(rps.tetap));
	int x2 = rps.xp - rps.l*cos(rps.tetap);
	int y2 = (SCREEN_HEIGHT - 1) - (rps.yp - rps.l*sin(rps.tetap));
	int x3 = rps.xp - rps.l*cos(rps.tetap) + 2*PADDLE_TIP_RADIUS*sin(rps.tetap);
	int y3 = (SCREEN_HEIGHT - 1) - (rps.yp - rps.l*sin(rps.tetap) - 2*PADDLE_TIP_RADIUS*cos(rps.tetap));
	int xc1 = rps.xp - rps.l*cos(rps.tetap) + PADDLE_TIP_RADIUS*sin(rps.tetap);
	int yc1 = (SCREEN_HEIGHT - 1) - (rps.yp - rps.l*sin(rps.tetap) - PADDLE_TIP_RADIUS*cos(rps.tetap));
	int xc2 = rps.xp + 0.5*rps.t*sin(rps.tetap);
	int yc2 = (SCREEN_HEIGHT - 1) - (rps.yp - 0.5*rps.t*cos(rps.tetap));
	int points[8] = {xp, yp, x1, y1, x3, y3, x2, y2};
	
	polygon(buf, 4,points, PADDLE_COLOR);
	circlefill(buf, xc1, yc1, PADDLE_TIP_RADIUS - 1, PADDLE_COLOR);
	circlefill(buf, xc2, yc2, 0.5*lps.t, PADDLE_COLOR);
}

//-----------------------------------------------------------------------------------
// La funzione write_dmiss scrive su buffer le deadline miss
//-----------------------------------------------------------------------------------
void write_dmiss(BITMAP* buf)
{
	char sB[80], sP[80], sG[80], sT[80];
	int i;
	int dmisstot = 0;
	
	sprintf(sP, "DEADLINE MISS TASKP : %i", tp[taskP_index].dmiss);
	sprintf(sG, "DEADLINE MISS TASKG : %i", tp[taskG_index].dmiss);
	sprintf(sT, "DEADLINE MISS TASKT : %i", tp[taskT_index].dmiss);
	textout_ex(buf, font, sP, X1, Y1 + 2*Y2, LETTER_COLOR, -1);
	textout_ex(buf, font, sG, X1, Y1 + 3*Y2, LETTER_COLOR, -1);
	textout_ex(buf, font, sT, X1, Y1 + 4*Y2, LETTER_COLOR, -1);
	
	for (i = 0; i < nb; i++){
		
		dmisstot += tp[i].dmiss;
	}
	
	sprintf(sB, "DEADLINE MISS TASKBALL : %i", dmisstot);
	textout_ex(buf, font, sB, X1, Y1 + 5*Y2, LETTER_COLOR, -1);
}

//-----------------------------------------------------------------------------------
// La funzione write_dmiss scrive su buffer il punteggio
//-----------------------------------------------------------------------------------
void write_points(BITMAP* buf)
{
	char s[80];
	int i;
	int ptot = 0;
	
	for ( i = 0; i < nb; i++){
		
		ptot += bs[i].bc;
	}
	
	sprintf(s, "POINTS : %i", ptot);
	textout_ex(buf, font, s, X1, Y1 + Y2, LETTER_COLOR, -1);
}

//-----------------------------------------------------------------------------------
// La funzione write_dmiss scrive su buffer la velocita' massima
// fra quelle di tutte le palline in gioco 
//-----------------------------------------------------------------------------------
void write_max_speed(BITMAP* buf)
{
	char s[80];
	int i, imax;
	imax = 0;

	for (i = 0; i < nb; i++){
		
		if ( ball_speed( i ) > ball_speed( imax ) ) imax = i;
	}

	sprintf(s, "MAX BALL SPEED : %.0f", ball_speed(imax));
	textout_ex(buf, font, s, X1, Y1 + 6*Y2, LETTER_COLOR, -1);
}

//-----------------------------------------------------------------------------------
// La funzione write_remaining_balls scrive su buffer
// quante palline mancano ancora da inserire  
//-----------------------------------------------------------------------------------
void write_remaining_balls(BITMAP* buf)
{
	char s[80];
	int rb;
	
	rb = MAX_BALLS - nb;
	sprintf(s, "REMAINING BALLS : %d", rb);
	textout_ex(buf, font, s, X1, Y1 + 7*Y2, LETTER_COLOR, -1);
}

//-----------------------------------------------------------------------------------
// La funzione draw_repulsors disegna i repulsori circolari 
//-----------------------------------------------------------------------------------
void draw_repulsors(BITMAP* buf)
{
	circlefill(buf, XR1, YBOX - YR1, R1_RADIUS, REPULSORS_COLOR);
	circlefill(buf, XR1, YBOX - YR1, R1_RADIUS - 10, SCREEN_COLOR);
	
	circlefill(buf, XR2, YBOX - YR2, R2_RADIUS, REPULSORS_COLOR);
	circlefill(buf, XR2, YBOX - YR2, R2_RADIUS - 10, SCREEN_COLOR);
	
	circlefill(buf, XBOX - XR2, YBOX - YR2, R2_RADIUS, REPULSORS_COLOR);
	circlefill(buf, XBOX - XR2, YBOX - YR2, R2_RADIUS - 10, SCREEN_COLOR);
}

//-----------------------------------------------------------------------------------
// La funzione draw_repulsors disegna i repulsori lineari
//-----------------------------------------------------------------------------------
void draw_s(BITMAP* buf)
{
	int x1l = XS;
	int y1l = (SCREEN_HEIGHT - 1) - YS;
	int x2l = XS + LS*cos(TETA_S);
	int y2l = (SCREEN_HEIGHT - 1) - (YS + LS*sin(TETA_S));
	int x3l = XS + LS*cos(TETA_S);
	int y3l = (SCREEN_HEIGHT - 1) - (YS + LS*sin(TETA_S) + TS);
	int x4l = XS;
	int y4l = (SCREEN_HEIGHT - 1) - (YS + TS);

	int x1r = XBOX -XS;
	int y1r = (SCREEN_HEIGHT - 1) - YS;
	int x2r = (XBOX - XS) - LS*cos(TETA_S);
	int y2r = (SCREEN_HEIGHT - 1) - (YS + LS*sin(TETA_S));
	int x3r = (XBOX - XS) - LS*cos(TETA_S);
	int y3r = (SCREEN_HEIGHT - 1) - (YS + LS*sin(TETA_S) + TS);
	int x4r = (XBOX - XS);
	int y4r = (SCREEN_HEIGHT - 1) - (YS + TS);

	int pointsl[8] = {x1l, y1l, x2l, y2l, x3l, y3l, x4l, y4l};
	int pointsr[8] = {x1r, y1r, x2r, y2r, x3r, y3r, x4r, y4r};
	
	polygon(buf, 4, pointsl, S_COLOR);
	polygon(buf, 4, pointsr, S_COLOR);
	circlefill(buf, XS, SCREEN_HEIGHT - 1 - YS - TS/2 , TS/2, S_COLOR);
	circlefill(buf, XS + LS*cos(TETA_S), SCREEN_HEIGHT - 1 - YS - TS/2 - LS*sin(TETA_S) , TS/2, S_COLOR);
	circlefill(buf, XBOX - XS, SCREEN_HEIGHT - 1 - YS - TS/2 , TS/2, S_COLOR);
	circlefill(buf, XBOX - XS - LS*cos(TETA_S), SCREEN_HEIGHT - 1 - YS - TS/2 - LS*sin(TETA_S) , TS/2, S_COLOR);
}

//-----------------------------------------------------------------------------------
// La funzione key_paddle interpreta comandi tastiera ed aggiorna i paddle 
//-----------------------------------------------------------------------------------
void key_paddle(float dt)
{
	if (!key[KEY_LEFT]){
		
		lps.tetap = LPADDLE_DOWN;								//se non viene premuta la freccia sinistra, il paddle sinistro rimane in condizione di riposo
    	lps.omegap = 0;
    }
    
	if ( (key[KEY_LEFT]) && ( lps.tetap < LPADDLE_UP ) ){		//se viene premuta la freccia sinistra, il paddle sinistro aggiorna la sua posizione
    	
		lps.tetap += PADDLE_VEL*dt;
    	lps.omegap = PADDLE_VEL;
	}
	
	if ( (key[KEY_LEFT]) && ( lps.tetap >= LPADDLE_UP ) ){		//se continuo a premere la freccia sinistra, ma sono gia' in posizione massima, rimango li 
    		
    	lps.tetap = LPADDLE_UP;
    	lps.omegap = 0;
	}
	
	if ( !key[KEY_RIGHT] ){
		
		rps.tetap = RPADDLE_DOWN;
		rps.omegap = 0;
	}
	
	if ( (key[KEY_RIGHT]) && (rps.tetap > RPADDLE_UP) ){
			
		rps.tetap -= PADDLE_VEL*dt;
		rps.omegap = PADDLE_VEL;
	}
	
	if ( (key[KEY_RIGHT]) && (rps.tetap <= RPADDLE_UP) ){
			
		rps.tetap = RPADDLE_UP;
		rps.omegap = 0;
	}
}

//-----------------------------------------------------------------------------------
// La funzione key_T interpreta comandi tastiera ed esegue operazioni taskT 
//-----------------------------------------------------------------------------------
void key_T(void)
{
	int k;
	if ( (key[KEY_SPACE] ) && (nb < MAX_BALLS) ){							//se premo il tasto SPACE e non sono ancora al numero massimo di palline, aggiungo ulteriore pallina

		task_create(taskB, nb, taskB_period, taskB_rdline, taskB_prio);
		nb++;
	}

	if (key[KEY_B]){													//se premo B palline diventano bianche
		
		for ( k = 0; k < MAX_BALLS; k++ ) {
			
			if ( bs[k].y + bs[k].r >= YF ) bs[k].color = 15;
		}
	}

	if( key[KEY_V]){
		
		for ( k = 0; k < MAX_BALLS; k++ ) {
			
			if( bs[k].y + bs[k].r >= YF ) bs[k].color = 10;
		}
	}
			
	if ( key[KEY_G]){
		
		for ( k = 0; k < MAX_BALLS; k++ ) {
			
			if ( bs[k].y + bs[k].r >= YF ) bs[k].color = 14;
		}
	}

	if ( key[KEY_R]){
		
		for ( k = 0; k < MAX_BALLS; k++ ) {
			
			if ( bs[k].y + bs[k].r >= YF ) bs[k].color = 12;
		}
	}
}
//---------------------------------------------------------------------------------------------------------------
//DEFINIZIONE FUNZIONI DEI TASK----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// La funzione taskB aggiorna lo stato della pallina i-esima
//-----------------------------------------------------------------------------------
void* taskB(void* arg)
{
    int i = get_task_index(arg);                                                                      																		
	int T = get_period(i);
	float scale_T = SCALEFACTOR_TIME;
    float dt = scale_T * (float)(T)/1000;
    float g = GRAVITY;
	
	bs[i].vx = VXI;
    
	periodic_task_init(i);
    while ( (bs[i].y + bs[i].r >= YF) && (!key[KEY_ESC]) ) {
    	
		bs[i].x += bs[i].vx * dt;							//Aggiornamento lungo x della posizione della pallina 
		bs[i].y += -0.5*g*dt*dt + bs[i].vy*dt;				//Aggiornamento lungo y della posizione della pallina 
		bs[i].vy += -g*dt;  								//Aggiornamento della vy 
		handle_bounce(XBOX, YBOX, i);						
		handle_bounce_r(XR1, YR1, R1_RADIUS, i);			
		handle_bounce_r(XR2, YR2, R2_RADIUS, i);			
		handle_bounce_r(XBOX - XR2, YR2, R2_RADIUS, i);		
		handle_bounce_s(i);													
		handle_bounce_bb(i);								
		speed_limiter(i);									
		
		dmiss_counter(i);
		wait_for_period(i);
    }
	bs[i].vx = 0;	
	bs[i].vy = 0;										
	bs[i].color = SCREEN_COLOR;								
	
	return NULL;
}

//-----------------------------------------------------------------------------------
// La funzione taskP aggiorna lo stato dei paddle
//-----------------------------------------------------------------------------------
void* taskP(void* arg)												
{
    int i = get_task_index(arg);                                                                      
	int T = get_period(i);
	float scale_T = SCALEFACTOR_TIME;
    float dt = scale_T * (float)(T)/1000;
	
	periodic_task_init(i);
    while ( !key[KEY_ESC] ) {
		
		key_paddle(dt);
		dmiss_counter(i);
		wait_for_period(i);
    }
    
    return NULL;
}

//-----------------------------------------------------------------------------------
// La funzione taskG aggiorna lo schermo
//-----------------------------------------------------------------------------------
void* taskG(void* arg)						
{
    int i = get_task_index(arg);
	BITMAP *buf;
	buf = create_bitmap(SCREEN_WIDTH, SCREEN_HEIGHT);
	
	periodic_task_init(i);
    while ( !key[KEY_ESC] ) {
        
		draw_bck(buf);
		draw_ball(buf);
		draw_lpaddle(buf);
		draw_rpaddle(buf);
		write_dmiss(buf);
		write_points(buf);
		write_max_speed(buf);
		write_remaining_balls(buf);
		blit(buf, screen, 0, 0, 0, 0, buf -> w, buf -> h);	//passaggio delle immagini da buffer a schermo 

		dmiss_counter(i);
		wait_for_period(i);
    }
	
    return NULL;
}

//-----------------------------------------------------------------------------------
// La funzione taskT si occupa dell'interfaccia con l'utente
//-----------------------------------------------------------------------------------
void* taskT(void* arg)
{
	int i = get_task_index(arg);

	periodic_task_init(i);
    while ( !key[KEY_ESC] ) {
		
		key_T();
		dmiss_counter(i);
		wait_for_period(i);
    }
	
    return NULL;
}
//---------------------------------------------------------------------------------------------------------------------------------------
//DEFINIZIONE FUNZIONE MAIN
int main()
{
	int i;
	
	screen_init();
	
	task_create(taskG, taskG_index, taskG_period, taskG_rdline, taskG_prio);
	task_create(taskP, taskP_index, taskP_period, taskP_rdline, taskP_prio);
	task_create(taskT, taskT_index, taskT_period, taskT_rdline, taskT_prio);
	
	wait_for_task_end(taskG_index);
	wait_for_task_end(taskP_index);
	wait_for_task_end(taskT_index);

	for ( i = 0; i < nb; i++){
		
		wait_for_task_end(i);
	}
	
	allegro_exit();
	
    return 0;
}
