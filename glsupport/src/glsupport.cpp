/********************************************************
 Stanford Driving Software
 Copyright (c) 2011 Stanford University
 All rights reserved.

 Redistribution and use in source and binary forms, with
 or without modification, are permitted provided that the
 following conditions are met:

 * Redistributions of source code must retain the above
 copyright notice, this list of conditions and the
 following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the
 following disclaimer in the documentation and/or other
 materials provided with the distribution.
 * The names of the contributors may not be used to endorse
 or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
 ********************************************************/

#include <cmath>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <angles/angles.h>
#include <glsupport/glsupport.h>

namespace glsupport {

static FontRenderer fr;

void draw_diamond(double x, double y, double r)
{
  glBegin(GL_POLYGON);
  glVertex2f(x + r, y);
  glVertex2f(x, y + r);
  glVertex2f(x - r, y);
  glVertex2f(x, y - r);
  glEnd();
}

void draw_arrow(double x1, double y1, double x2, double y2, double head_width, double head_length)
{
  double angle, ct, st;

  glPushMatrix();
  glBegin(GL_LINES);
  glVertex2f(x1, y1);
  glVertex2f(x2, y2);
  glEnd();

  glTranslatef(0, 0, 0.03);
  angle = atan2(y2 - y1, x2 - x1);
  ct = cos(angle);
  st = sin(angle);
  glBegin(GL_POLYGON);
  glVertex2f(x2, y2);
  glVertex2f(x2 - head_length * ct + head_width * st, y2 - head_length * st - head_width * ct);
  glVertex2f(x2 - head_length * ct - head_width * st, y2 - head_length * st + head_width * ct);
  glEnd();
  glPopMatrix();
}

void draw_arrowhead(double x, double y, double angle)
{
  double ct, st, l = 2, l2 = 0.5;

  ct = cos(angle);
  st = sin(angle);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x - l * ct + l2 * st, y - l * st - l2 * ct);
  glVertex2f(x - l * ct - l2 * st, y - l * st + l2 * ct);
  glEnd();
}

void draw_observed_car(double x, double y, double theta,
                       double w, double l,
                       int id,
                       double v,
                       bool draw_flag,
                       double x_var, double y_var,
                       int /*tracking_state*/,
                       int /*lane*/,
                       double confidence,
                       bool published,
                       double camera_pan,
                       double proba_no_turn_signal,
                       double proba_turn_signal_left,
                       double proba_turn_signal_right,
                       bool display_turn_signal) // TODO: use or remove?!?
{
  const int num_lines = 5;
  char line1[100], line2[100], line3[100], line4[100], line5[100], line6[100], line7[100];
  char *text[7];
//  int color = abs(id) % 13;

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(2);

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(angles::to_degrees(theta), 0, 0, 1);

  //if (published) glColor4f(0, 1, 0, 0.7);
  //else glColor4f(1, 1, 1, 0.7);

  // confidence currently corresponds to the number of observations
   if(confidence < 3.)
     // obstacles that were not tracked long enough are displayed grey
     glColor4f(1, 1, 1, 0.7);
   else
     // all others are displayed red
     glColor4f(0, 1, 0, 0.7);

  // draw car outline
  glBegin(GL_POLYGON);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(l / 4.0, 0);
  glVertex2f(l * 3 / 4.0, 0);
  glEnd();
  glPopMatrix();

  // pose accuracy
  glPushMatrix();
  glTranslatef(x, y, 0);
  glColor4f(0.6, 0.6, 0, 0.5);
  draw_ellipse(0, 0, sqrt(x_var), sqrt(y_var), 0);
  glPopMatrix();

  // Turn signal
  if (display_turn_signal) {
    glPushMatrix();
    glTranslatef(x, y, 0);
    glRotatef(angles::to_degrees(theta - M_PI_2), 0, 0, 1);
    glColor3f(1.0, 0.0, 0.0);

    draw_circle(0, -l / 2 - 0.5, proba_no_turn_signal, 1);
    draw_circle(-w / 2 - 0.5, -l / 2 - 0.5, proba_turn_signal_left, 1);
    draw_circle(w / 2 + 0.5, -l / 2 - 0.5, proba_turn_signal_right, 1);
    glPopMatrix();
  }

  // draw info
  if (draw_flag) {
    /* TODO: Something is weird here:
     * the position of the text depends on the code AFTER the call of glPopMatrix()
     * Shouldn't it be glTranslatef(x, y, 0)?

    glPushMatrix();
    glTranslatef(0, 0, 1.0);
    char buf[255];
    //sprintf(buf, "%d", (int)confidence);
    sprintf(buf, "%.2f", v);
    glColor3f(1, 0, 0);
    render_stroke_text_centered_2D(x, y, 1, buf);
    glPopMatrix();
    */

    sprintf(line1, "ID:  %d", id);
    sprintf(line2, "X:   %.2f", x);
    sprintf(line3, "Y:   %.2f", y);
    sprintf(line4, "VEL: %.2f m/s", v);
    sprintf(line5, "DIR: %.2f", theta);
    text[0] = line1;
    text[1] = line2;
    text[2] = line3;
    text[3] = line4;
    text[4] = line5;
    text[5] = line6;
    text[6] = line7;

    glPushMatrix();
    glTranslatef(x, y, 0);


    //show a vertical flag above the obstacle
    glRotatef(camera_pan + 90., 0, 0, 1);
    glTranslatef(w / 2., 0., 1.);
    glRotatef(90., 1., 0, 0);

    //show a flat flag right of the obstacle
    //glRotatef(angles::to_degrees(theta - M_PI_2), 0, 0, 1);
    //glTranslatef(w / 2., 0., 0.);


    glScalef(.5,.5,.5);

    //font size fixed to 6 points
    float dh = 1.;

    //make a semi-transparent background box
    glColor4f (1., .9, 1., .6);
    glBegin (GL_TRIANGLE_STRIP);
    glVertex3f (0., 0., -.01);
    glVertex3f (0., (num_lines + .4) * dh, -.01);
    glVertex3f (8., 0., 0.);
    glVertex3f (8., (num_lines + .4) * dh, -.01);
    glEnd ();

    //draw the text
    glColor4f(0.,0.,0.,1.);
    glTranslatef(.2, .2, 0.);

    for (int i = 0; i < num_lines; i++)
      if (text[num_lines - i - 1] != NULL)
        render_stroke_text_2D(0., i * dh, 6. * dh, text[num_lines - i - 1]);

    glPopMatrix();
  }

  glLineWidth(1);

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void set_display_mode_2D(int w, int h)
{
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //  gluOrtho2D(0.0, (GLfloat)w, 0.0, (GLfloat)h);
  glOrtho(0.0, (GLfloat) w, 0.0, (GLfloat) h, -1000, 1000);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void set_display_mode_3D(int w, int h, float fovy, float near, float far)
{
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovy, w / (float) h, near, far);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void draw_circle(double x, double y, double r, bool filled)
{
  int i;
  double angle;

  if (filled) {
    glBegin(GL_TRIANGLE_FAN);
  }
  else {
    glBegin(GL_LINE_LOOP);
  }

  for (i = 0; i < 20; i++) {
    angle = i / 20.0 * M_PI * 2;
    glVertex2f(x + r * cos(angle), y + r * sin(angle));
  }
  glEnd();
}

void draw_dashed_circle(double x, double y, double r)
{
  int i;
  double angle;
  double d = 1 / 40.0 * M_PI * 2;

  glBegin(GL_LINES);
  for (i = 0; i < 20; i++) {
    angle = i / 20.0 * M_PI * 2;
    glVertex2f(x + r * cos(angle), y + r * sin(angle));
    glVertex2f(x + r * cos(angle + d), y + r * sin(angle + d));
  }
  glEnd();
}

void draw_ellipse(double x, double y, double rx, double ry, bool filled)
{
  int i;
  double angle;

  if (filled) {
    glBegin(GL_TRIANGLE_FAN);
  }
  else {
    glBegin(GL_LINE_LOOP);
  }

  for (i = 0; i < 20; i++) {
    angle = i / 20.0 * M_PI * 2;
    glVertex2f(x + rx * cos(angle), y + ry * sin(angle));
  }
  glEnd();
}

void render_stroke_text_2D(float x, float y, float size, const char* string)
{
  uint32_t old_size = fr.fontSize();
  fr.setFontSize(size);
  fr.drawString2D(string, x, y);
  fr.setFontSize(old_size);
}

void render_stroke_text_centered_2D(float x, float y, float size, const char* string)
{
  float w, h;
  fr.stringBBox(string, w, h);
  render_stroke_text_2D(x - w / 2, y - h / 2, size, string);
}

void renderBitmapString(float x, float y, const char* string)
{
  fr.drawString2D(string, x, y);
//  fr.drawPixmapString2D(string, x, y);
}

int bitmapStringWidth(const char* text)
{
  return fr.stringWidth(text);
}

void renderBitmapStringCentered(float x, float y, const char* string)
{
  int vert_offset = -5;

  fr.drawPixmapString2D(string, x-bitmapStringWidth(string)/2.0, y+vert_offset);
}

void draw_scale(int window_width, double map_zoom)
{
  char str[50];

  glBegin(GL_LINES);
  glVertex2f(window_width - 220, 17);
  glVertex2f(window_width - 20, 17);
  glVertex2f(window_width - 220, 22);
  glVertex2f(window_width - 220, 12);
  glVertex2f(window_width - 20, 22);
  glVertex2f(window_width - 20, 12);
  glEnd();
  glPushMatrix();
  sprintf(str, "%.2f meters", 200 / map_zoom);
  glTranslatef(window_width - 120 - fr.stringWidth(str) * 0.15 / 2.0, 22, 0);
  glScalef(0.15, 0.15, 1);
  fr.drawString2D(str);
  glPopMatrix();
}

void draw_distance_rings(double x, double y, double theta, int max_distance, int distance_increment)
{
  int i, j;
  char buf[255];
  float angle;
  glLineWidth(0.5);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  uint32_t old_size = fr.fontSize();
  fr.setFontSize(0.3);
  // draw radius
  glPushMatrix();
  glRotatef(angles::to_degrees(theta), 0, 0, 1);
  glTranslatef(x, y, 0);
  glLineWidth(2.0);
  for (i = distance_increment; i <= max_distance; i += distance_increment) {
    glColor4f(0.5, 0.5, 0.5, 1.0);
    sprintf(buf, "%d", i);
    fr.drawString2D(buf, i, 0);
    glBegin(GL_LINE_LOOP);
    for (j = 0; j < 100; j++) {
      angle = j / 100.0 * M_PI * 2;
      glVertex3f(i * cos(angle), i * sin(angle), 0);
    }
    glEnd();
  }
  glPopMatrix();
  fr.setFontSize(old_size);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void draw_vehicle_outline(double length, double width, double height,
                          double wheel_base,
                          double origin_to_front_axle_dist,
                          double front_axle_to_front_bumper_dist,
                          double track_width,
                          double wheel_radius, double wheel_angle)
{
  double l = length, w = width;
  double d;

  d = origin_to_front_axle_dist + front_axle_to_front_bumper_dist;
  glBegin(GL_POLYGON);
  glVertex2f(d - l, -w / 2.0);
  glVertex2f(d - l, w / 2.0);
  glVertex2f(d, w / 2.0);
  glVertex2f(d, -w / 2.0);
  glEnd();

  glColor3f(0, 0, 0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(d - l, -w / 2.0);
  glVertex2f(d - l, w / 2.0);
  glVertex2f(d, w / 2.0);
  glVertex2f(d, -w / 2.0);
  glEnd();

  /* draw the vehicle */
  d = origin_to_front_axle_dist - wheel_base;

  glBegin(GL_LINES);
  glVertex2f(d, 0);
  glVertex2f(origin_to_front_axle_dist, 0);
  glVertex2f(d, -track_width / 2.0);
  glVertex2f(d, track_width / 2.0);
  glVertex2f(origin_to_front_axle_dist, -track_width / 2.0);
  glVertex2f(origin_to_front_axle_dist, track_width / 2.0);
  glVertex2f(d - wheel_radius, -track_width / 2.0);
  glVertex2f(d + wheel_radius, -track_width / 2.0);
  glVertex2f(d - wheel_radius, track_width / 2.0);
  glVertex2f(d + wheel_radius, track_width / 2.0);
  glEnd();

  glPushMatrix();
  glTranslatef(origin_to_front_axle_dist, -track_width / 2.0, 0.0);
  glRotatef(angles::to_degrees(wheel_angle), 0, 0, 1);
  glBegin(GL_LINES);
  glVertex2f(-wheel_radius, 0);
  glVertex2f(wheel_radius, 0);
  glEnd();
  glPopMatrix();

  glPushMatrix();
  glTranslatef(origin_to_front_axle_dist, track_width / 2.0, 0.0);
  glRotatef(angles::to_degrees(wheel_angle), 0, 0, 1);
  glBegin(GL_LINES);
  glVertex2f(-wheel_radius, 0);
  glVertex2f(wheel_radius, 0);
  glEnd();
  glPopMatrix();
}

void draw_passat_simple(double length, double width, double height,
                        double wheel_base, double track_width,
                        double wheel_radius, double wheel_angle)
{
  int j;
  double angle;

#define PASSAT_TRUNK_HEIGHT  0.8
#define PASSAT_BOTTOM_HEIGHT 0.3
#define PASSAT_FRONT_HEIGHT  0.6
#define PASSAT_TOP_WIDTH     1.4
#define EPS 0.01 

  // bottom
  glBegin(GL_POLYGON);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, 0);
  glEnd();
  // sides
  glBegin(GL_QUAD_STRIP);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, PASSAT_TRUNK_HEIGHT);

  glVertex3f(-0.2, -PASSAT_TOP_WIDTH / 2.0, 0);
  glVertex3f(-0.2, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);

  glVertex3f(1.5, -PASSAT_TOP_WIDTH / 2.0, 0);
  glVertex3f(1.5, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);

  glVertex3f(2.5, -width / 2.0, 0);
  glVertex3f(2.5, -width / 2.0, PASSAT_TRUNK_HEIGHT);

  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, PASSAT_FRONT_HEIGHT);

  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, PASSAT_FRONT_HEIGHT);

  glVertex3f(2.5, width / 2.0, 0);
  glVertex3f(2.5, width / 2.0, PASSAT_TRUNK_HEIGHT);

  glVertex3f(1.5, PASSAT_TOP_WIDTH / 2.0, 0);
  glVertex3f(1.5, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);

  glVertex3f(-0.2, PASSAT_TOP_WIDTH / 2.0, 0);
  glVertex3f(-0.2, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);

  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, PASSAT_TRUNK_HEIGHT);

  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, PASSAT_TRUNK_HEIGHT);
  glEnd();

  // roof
  glBegin(GL_QUAD_STRIP);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, PASSAT_TRUNK_HEIGHT - EPS);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, PASSAT_TRUNK_HEIGHT - EPS);

  glVertex3f(-0.2, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT - EPS);
  glVertex3f(-0.2, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT - EPS);

  glVertex3f(1.5, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT - EPS);
  glVertex3f(1.5, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT - EPS);

  glVertex3f(2.5, -PASSAT_TOP_WIDTH / 2.0, PASSAT_TRUNK_HEIGHT - EPS);
  glVertex3f(2.5, PASSAT_TOP_WIDTH / 2.0, PASSAT_TRUNK_HEIGHT - EPS);

  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, PASSAT_FRONT_HEIGHT - EPS);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, PASSAT_FRONT_HEIGHT - EPS);

  glEnd();

  //edges: left
  glColor3f(0, 0, 0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-0.2, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(1.5, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(2.5, -width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, PASSAT_FRONT_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, 0);
  glEnd();
  //edges:right
  glBegin(GL_LINE_LOOP);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-0.2, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(1.5, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(2.5, width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, PASSAT_FRONT_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, 0);
  glEnd();
  // edges:roof
  glBegin(GL_LINES);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0, -width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0, width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-0.2, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(-0.2, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(1.5, -PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(1.5, PASSAT_TOP_WIDTH / 2.0, height - PASSAT_BOTTOM_HEIGHT);
  glVertex3f(2.5, -width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(2.5, width / 2.0, PASSAT_TRUNK_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, PASSAT_FRONT_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, PASSAT_FRONT_HEIGHT);
  glVertex3f(-(length - wheel_base) / 2.0 + length, -width / 2.0, 0);
  glVertex3f(-(length - wheel_base) / 2.0 + length, width / 2.0, 0);
  glEnd();

  /* wheels */
  glBegin(GL_LINE_LOOP);
  for (j = 0; j < 8; j++) {
    angle = j / 8.0 * M_PI * 2;
    glVertex3f(wheel_radius * cos(angle), -track_width / 2.0, wheel_radius * sin(angle) - 0.2);
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for (j = 0; j < 8; j++) {
    angle = j / 8.0 * M_PI * 2;
    glVertex3f(wheel_radius * cos(angle), track_width / 2.0, wheel_radius * sin(angle) - 0.2);
  }
  glEnd();

  glPushMatrix();
  glTranslatef(wheel_base, -track_width / 2.0, 0.0);
  glRotatef(angles::to_degrees(wheel_angle), 0, 0, 1);
  glBegin(GL_LINE_LOOP);
  for (j = 0; j < 8; j++) {
    angle = j / 8.0 * M_PI * 2;
    glVertex3f(wheel_radius * cos(angle), 0, wheel_radius * sin(angle) - 0.2);
  }
  glEnd();
  glPopMatrix();

  glPushMatrix();
  glTranslatef(wheel_base, track_width / 2.0, 0.0);
  glRotatef(angles::to_degrees(wheel_angle), 0, 0, 1);
  glBegin(GL_LINE_LOOP);
  for (j = 0; j < 8; j++) {
    angle = j / 8.0 * M_PI * 2;
    glVertex3f(wheel_radius * cos(angle), 0, wheel_radius * sin(angle) - 0.2);
  }
  glEnd();
  glPopMatrix();
}

inline void RTOGA(double x, double min, double max, double a)
{
  double temp = ((x) - (min)) / ((max) - (min));

  if (temp < 0) glColor4f(0, 1, 0, a);
  else if (temp > 1.0) glColor4f(1, 0, 0, a);
  else glColor4f(temp, 1 - temp, 0, a);
}

// Draw an X-Y-Z Frame. The red arrow corresponds to the X-Axis,
// green to the Y-Axis, and blue to the Z-Axis.
void draw_coordinate_frame(double scale)
{
  double a_axisThicknessScale = 3 * scale;
  const bool modify_material_state_ = true;

  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  // Triangle vertices:
  static int nTriangles = 8;

  static float triangle_vertices[72] = { 0.000000f, 0.040000f, -0.800000f, 0.028284f, 0.028284f, -0.800000f, 0.000000f, 0.000000f, -1.000000f, 0.028284f,
      0.028284f, -0.800000f, 0.040000f, 0.000000f, -0.800000f, 0.000000f, 0.000000f, -1.000000f, 0.040000f, 0.000000f, -0.800000f, 0.028284f, -0.028284f,
      -0.800000f, 0.000000f, 0.000000f, -1.000000f, 0.028284f, -0.028284f, -0.800000f, 0.000000f, -0.040000f, -0.800000f, 0.000000f, 0.000000f, -1.000000f,
      0.000000f, -0.040000f, -0.800000f, -0.028284f, -0.028284f, -0.800000f, 0.000000f, 0.000000f, -1.000000f, -0.028284f, -0.028284f, -0.800000f, -0.040000f,
      0.000000f, -0.800000f, 0.000000f, 0.000000f, -1.000000f, -0.040000f, 0.000000f, -0.800000f, -0.028284f, 0.028284f, -0.800000f, 0.000000f, 0.000000f,
      -1.000000f, -0.028284f, 0.028284f, -0.800000f, 0.000000f, 0.040000f, -0.800000f, 0.000000f, 0.000000f, -1.000000f };

  // Triangle normals:
  static float triangle_normals[72] = { 0.000000f, 0.980581f, -0.196116f, 0.693375f, 0.693375f, -0.196116f, 0.357407f, 0.862856f, -0.357407f, 0.693375f,
      0.693375f, -0.196116f, 0.980581f, 0.000000f, -0.196116f, 0.862856f, 0.357407f, -0.357407f, 0.980581f, 0.000000f, -0.196116f, 0.693375f, -0.693375f,
      -0.196116f, 0.862856f, -0.357407f, -0.357407f, 0.693375f, -0.693375f, -0.196116f, 0.000000f, -0.980581f, -0.196116f, 0.357407f, -0.862856f, -0.357407f,
      0.000000f, -0.980581f, -0.196116f, -0.693375f, -0.693375f, -0.196116f, -0.357407f, -0.862856f, -0.357407f, -0.693375f, -0.693375f, -0.196116f, -0.980581f,
      0.000000f, -0.196116f, -0.862856f, -0.357407f, -0.357407f, -0.980581f, 0.000000f, -0.196116f, -0.693375f, 0.693375f, -0.196116f, -0.862856f, 0.357407f,
      -0.357407f, -0.693375f, 0.693375f, -0.196116f, 0.000000f, 0.980581f, -0.196116f, -0.357407f, 0.862856f, -0.357407f };

  // Quad vertices:
  static int nQuads = 16;

  static float quad_vertices[192] = { 0.000000f, 0.010000f, 0.000000f, 0.007000f, 0.007000f, 0.000000f, 0.007000f, 0.007000f, -0.800000f, 0.000000f, 0.010000f,
      -0.800000f, 0.000000f, -0.010000f, 0.000000f, -0.007000f, -0.007000f, 0.000000f, -0.007000f, -0.007000f, -0.800000f, 0.000000f, -0.010000f, -0.800000f,
      -0.007000f, -0.007000f, 0.000000f, -0.010000f, 0.000000f, 0.000000f, -0.010000f, 0.000000f, -0.800000f, -0.007000f, -0.007000f, -0.800000f, -0.010000f,
      0.000000f, 0.000000f, -0.007000f, 0.007000f, 0.000000f, -0.007000f, 0.007000f, -0.800000f, -0.010000f, 0.000000f, -0.800000f, -0.007000f, 0.007000f,
      0.000000f, 0.000000f, 0.010000f, 0.000000f, 0.000000f, 0.010000f, -0.800000f, -0.007000f, 0.007000f, -0.800000f, 0.007000f, 0.007000f, 0.000000f,
      0.010000f, 0.000000f, 0.000000f, 0.010000f, 0.000000f, -0.800000f, 0.007000f, 0.007000f, -0.800000f, 0.010000f, 0.000000f, 0.000000f, 0.007000f,
      -0.007000f, 0.000000f, 0.007000f, -0.007000f, -0.800000f, 0.010000f, 0.000000f, -0.800000f, 0.007000f, -0.007000f, 0.000000f, 0.000000f, -0.010000f,
      0.000000f, 0.000000f, -0.010000f, -0.800000f, 0.007000f, -0.007000f, -0.800000f, -0.007000f, 0.007000f, -0.800000f, -0.028284f, 0.028284f, -0.800000f,
      -0.040000f, 0.000000f, -0.800000f, -0.010000f, 0.000000f, -0.800000f, -0.010000f, 0.000000f, -0.800000f, -0.040000f, 0.000000f, -0.800000f, -0.028284f,
      -0.028284f, -0.800000f, -0.007000f, -0.007000f, -0.800000f, -0.007000f, -0.007000f, -0.800000f, -0.028284f, -0.028284f, -0.800000f, 0.000000f, -0.040000f,
      -0.800000f, 0.000000f, -0.010000f, -0.800000f, 0.000000f, -0.010000f, -0.800000f, 0.000000f, -0.040000f, -0.800000f, 0.028284f, -0.028284f, -0.800000f,
      0.007000f, -0.007000f, -0.800000f, 0.028284f, -0.028284f, -0.800000f, 0.040000f, 0.000000f, -0.800000f, 0.010000f, 0.000000f, -0.800000f, 0.007000f,
      -0.007000f, -0.800000f, 0.040000f, 0.000000f, -0.800000f, 0.028284f, 0.028284f, -0.800000f, 0.007000f, 0.007000f, -0.800000f, 0.010000f, 0.000000f,
      -0.800000f, 0.007000f, 0.007000f, -0.800000f, 0.028284f, 0.028284f, -0.800000f, 0.000000f, 0.040000f, -0.800000f, 0.000000f, 0.010000f, -0.800000f,
      0.000000f, 0.010000f, -0.800000f, 0.000000f, 0.040000f, -0.800000f, -0.028284f, 0.028284f, -0.800000f, -0.007000f, 0.007000f, -0.800000f };

  // Quad normals:
  static float quad_normals[192] = { 0.000000f, 1.000000f, 0.000000f, 0.707107f, 0.707107f, 0.000000f, 0.707107f, 0.707107f, 0.000000f, 0.000000f, 1.000000f,
      0.000000f, 0.000000f, -1.000000f, 0.000000f, -0.707107f, -0.707107f, 0.000000f, -0.707107f, -0.707107f, 0.000000f, 0.000000f, -1.000000f, 0.000000f,
      -0.707107f, -0.707107f, 0.000000f, -1.000000f, 0.000000f, 0.000000f, -1.000000f, 0.000000f, 0.000000f, -0.707107f, -0.707107f, 0.000000f, -1.000000f,
      0.000000f, 0.000000f, -0.707107f, 0.707107f, 0.000000f, -0.707107f, 0.707107f, 0.000000f, -1.000000f, 0.000000f, 0.000000f, -0.707107f, 0.707107f,
      0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, -0.707107f, 0.707107f, 0.000000f, 0.707107f, 0.707107f, 0.000000f, 1.000000f,
      0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.707107f, 0.707107f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.707107f, -0.707107f, 0.000000f,
      0.707107f, -0.707107f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.707107f, -0.707107f, 0.000000f, 0.000000f, -1.000000f, 0.000000f, 0.000000f,
      -1.000000f, 0.000000f, 0.707107f, -0.707107f, 0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f,
      1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f,
      0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
      0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f,
      1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f,
      0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
      0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f,
      1.000000f, 0.000000f, 0.000000f, 1.000000f };

  glEnableClientState(GL_NORMAL_ARRAY);
  glEnableClientState(GL_VERTEX_ARRAY);

  // Set up nice color-tracking
  if (modify_material_state_) {
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  int k;
  for (k = 0; k < 3; k++) {

    glPushMatrix();

    // Rotate to the appropriate axis
    if (k == 0) {
      glRotatef(-90.0, 0, 1, 0);
      glColor3f(1.0f, 0.0f, 0.0f);
    }
    else if (k == 1) {
      glRotatef(90.0, 1, 0, 0);
      glColor3f(0.0f, 1.0f, 0.0f);
    }
    else {
      glRotatef(180.0, 1, 0, 0);
      glColor3f(0.0f, 0.0f, 1.0f);
    }

    glScaled(a_axisThicknessScale, a_axisThicknessScale, scale);

    glVertexPointer(3, GL_FLOAT, 0, triangle_vertices);
    glNormalPointer(GL_FLOAT, 0, triangle_normals);
    glDrawArrays(GL_TRIANGLES, 0, nTriangles * 3);

    glVertexPointer(3, GL_FLOAT, 0, quad_vertices);
    glNormalPointer(GL_FLOAT, 0, quad_normals);
    glDrawArrays(GL_QUADS, 0, nQuads * 4);

    glPopMatrix();
  }

  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_arrowhead_2D(double x, double y, double angle)
{
  double ct, st, l = 2, l2 = 0.5;

  ct = cos(angle);
  st = sin(angle);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x - l * ct + l2 * st, y - l * st - l2 * ct);
  glVertex2f(x - l * ct - l2 * st, y - l * st + l2 * ct);
  glEnd();
}

float dgc_colormap_hsv[128][3] = { { 1, 0, 0 }, { 1, 0.046875, 0 }, { 1, 0.09375, 0 }, { 1, 0.14062, 0 }, { 1, 0.1875, 0 }, { 1, 0.23438, 0 },
    { 1, 0.28125, 0 }, { 1, 0.32812, 0 }, { 1, 0.375, 0 }, { 1, 0.42188, 0 }, { 1, 0.46875, 0 }, { 1, 0.51562, 0 }, { 1, 0.5625, 0 }, { 1, 0.60938, 0 }, { 1,
        0.65625, 0 }, { 1, 0.70312, 0 }, { 1, 0.75, 0 }, { 1, 0.79688, 0 }, { 1, 0.84375, 0 }, { 1, 0.89062, 0 }, { 1, 0.9375, 0 }, { 1, 0.98438, 0 }, {
        0.96875, 1, 0 }, { 0.92188, 1, 0 }, { 0.875, 1, 0 }, { 0.82812, 1, 0 }, { 0.78125, 1, 0 }, { 0.73438, 1, 0 }, { 0.6875, 1, 0 }, { 0.64062, 1, 0 }, {
        0.59375, 1, 0 }, { 0.54688, 1, 0 }, { 0.5, 1, 0 }, { 0.45312, 1, 0 }, { 0.40625, 1, 0 }, { 0.35938, 1, 0 }, { 0.3125, 1, 0 }, { 0.26562, 1, 0 }, {
        0.21875, 1, 0 }, { 0.17188, 1, 0 }, { 0.125, 1, 0 }, { 0.078125, 1, 0 }, { 0.03125, 1, 0 }, { 0, 1, 0.015625 }, { 0, 1, 0.0625 }, { 0, 1, 0.10938 }, {
        0, 1, 0.15625 }, { 0, 1, 0.20312 }, { 0, 1, 0.25 }, { 0, 1, 0.29688 }, { 0, 1, 0.34375 }, { 0, 1, 0.39062 }, { 0, 1, 0.4375 }, { 0, 1, 0.48438 }, { 0,
        1, 0.53125 }, { 0, 1, 0.57812 }, { 0, 1, 0.625 }, { 0, 1, 0.67188 }, { 0, 1, 0.71875 }, { 0, 1, 0.76562 }, { 0, 1, 0.8125 }, { 0, 1, 0.85938 }, { 0, 1,
        0.90625 }, { 0, 1, 0.95312 }, { 0, 1, 1 }, { 0, 0.95312, 1 }, { 0, 0.90625, 1 }, { 0, 0.85938, 1 }, { 0, 0.8125, 1 }, { 0, 0.76562, 1 },
    { 0, 0.71875, 1 }, { 0, 0.67188, 1 }, { 0, 0.625, 1 }, { 0, 0.57812, 1 }, { 0, 0.53125, 1 }, { 0, 0.48438, 1 }, { 0, 0.4375, 1 }, { 0, 0.39062, 1 }, { 0,
        0.34375, 1 }, { 0, 0.29688, 1 }, { 0, 0.25, 1 }, { 0, 0.20312, 1 }, { 0, 0.15625, 1 }, { 0, 0.10938, 1 }, { 0, 0.0625, 1 }, { 0, 0.015625, 1 }, {
        0.03125, 0, 1 }, { 0.078125, 0, 1 }, { 0.125, 0, 1 }, { 0.17188, 0, 1 }, { 0.21875, 0, 1 }, { 0.26562, 0, 1 }, { 0.3125, 0, 1 }, { 0.35938, 0, 1 }, {
        0.40625, 0, 1 }, { 0.45312, 0, 1 }, { 0.5, 0, 1 }, { 0.54688, 0, 1 }, { 0.59375, 0, 1 }, { 0.64062, 0, 1 }, { 0.6875, 0, 1 }, { 0.73438, 0, 1 }, {
        0.78125, 0, 1 }, { 0.82812, 0, 1 }, { 0.875, 0, 1 }, { 0.92188, 0, 1 }, { 0.96875, 0, 1 }, { 1, 0, 0.98438 }, { 1, 0, 0.9375 }, { 1, 0, 0.89062 }, { 1,
        0, 0.84375 }, { 1, 0, 0.79688 }, { 1, 0, 0.75 }, { 1, 0, 0.70312 }, { 1, 0, 0.65625 }, { 1, 0, 0.60938 }, { 1, 0, 0.5625 }, { 1, 0, 0.51562 }, { 1, 0,
        0.46875 }, { 1, 0, 0.42188 }, { 1, 0, 0.375 }, { 1, 0, 0.32812 }, { 1, 0, 0.28125 }, { 1, 0, 0.23438 }, { 1, 0, 0.1875 }, { 1, 0, 0.14062 }, { 1, 0,
        0.09375 }, { 1, 0, 0.046875 } };

float dgc_colormap_hot[128][3] = { { 0.020833, 0, 0 }, { 0.041667, 0, 0 }, { 0.0625, 0, 0 }, { 0.083333, 0, 0 }, { 0.10417, 0, 0 }, { 0.125, 0, 0 }, { 0.14583,
    0, 0 }, { 0.16667, 0, 0 }, { 0.1875, 0, 0 }, { 0.20833, 0, 0 }, { 0.22917, 0, 0 }, { 0.25, 0, 0 }, { 0.27083, 0, 0 }, { 0.29167, 0, 0 }, { 0.3125, 0, 0 }, {
    0.33333, 0, 0 }, { 0.35417, 0, 0 }, { 0.375, 0, 0 }, { 0.39583, 0, 0 }, { 0.41667, 0, 0 }, { 0.4375, 0, 0 }, { 0.45833, 0, 0 }, { 0.47917, 0, 0 }, { 0.5, 0,
    0 }, { 0.52083, 0, 0 }, { 0.54167, 0, 0 }, { 0.5625, 0, 0 }, { 0.58333, 0, 0 }, { 0.60417, 0, 0 }, { 0.625, 0, 0 }, { 0.64583, 0, 0 }, { 0.66667, 0, 0 }, {
    0.6875, 0, 0 }, { 0.70833, 0, 0 }, { 0.72917, 0, 0 }, { 0.75, 0, 0 }, { 0.77083, 0, 0 }, { 0.79167, 0, 0 }, { 0.8125, 0, 0 }, { 0.83333, 0, 0 }, { 0.85417,
    0, 0 }, { 0.875, 0, 0 }, { 0.89583, 0, 0 }, { 0.91667, 0, 0 }, { 0.9375, 0, 0 }, { 0.95833, 0, 0 }, { 0.97917, 0, 0 }, { 1, 0, 0 }, { 1, 0.020833, 0 }, { 1,
    0.041667, 0 }, { 1, 0.0625, 0 }, { 1, 0.083333, 0 }, { 1, 0.10417, 0 }, { 1, 0.125, 0 }, { 1, 0.14583, 0 }, { 1, 0.16667, 0 }, { 1, 0.1875, 0 }, { 1,
    0.20833, 0 }, { 1, 0.22917, 0 }, { 1, 0.25, 0 }, { 1, 0.27083, 0 }, { 1, 0.29167, 0 }, { 1, 0.3125, 0 }, { 1, 0.33333, 0 }, { 1, 0.35417, 0 },
    { 1, 0.375, 0 }, { 1, 0.39583, 0 }, { 1, 0.41667, 0 }, { 1, 0.4375, 0 }, { 1, 0.45833, 0 }, { 1, 0.47917, 0 }, { 1, 0.5, 0 }, { 1, 0.52083, 0 }, { 1,
        0.54167, 0 }, { 1, 0.5625, 0 }, { 1, 0.58333, 0 }, { 1, 0.60417, 0 }, { 1, 0.625, 0 }, { 1, 0.64583, 0 }, { 1, 0.66667, 0 }, { 1, 0.6875, 0 }, { 1,
        0.70833, 0 }, { 1, 0.72917, 0 }, { 1, 0.75, 0 }, { 1, 0.77083, 0 }, { 1, 0.79167, 0 }, { 1, 0.8125, 0 }, { 1, 0.83333, 0 }, { 1, 0.85417, 0 }, { 1,
        0.875, 0 }, { 1, 0.89583, 0 }, { 1, 0.91667, 0 }, { 1, 0.9375, 0 }, { 1, 0.95833, 0 }, { 1, 0.97917, 0 }, { 1, 1, 0 }, { 1, 1, 0.03125 },
    { 1, 1, 0.0625 }, { 1, 1, 0.09375 }, { 1, 1, 0.125 }, { 1, 1, 0.15625 }, { 1, 1, 0.1875 }, { 1, 1, 0.21875 }, { 1, 1, 0.25 }, { 1, 1, 0.28125 }, { 1, 1,
        0.3125 }, { 1, 1, 0.34375 }, { 1, 1, 0.375 }, { 1, 1, 0.40625 }, { 1, 1, 0.4375 }, { 1, 1, 0.46875 }, { 1, 1, 0.5 }, { 1, 1, 0.53125 },
    { 1, 1, 0.5625 }, { 1, 1, 0.59375 }, { 1, 1, 0.625 }, { 1, 1, 0.65625 }, { 1, 1, 0.6875 }, { 1, 1, 0.71875 }, { 1, 1, 0.75 }, { 1, 1, 0.78125 }, { 1, 1,
        0.8125 }, { 1, 1, 0.84375 }, { 1, 1, 0.875 }, { 1, 1, 0.90625 }, { 1, 1, 0.9375 }, { 1, 1, 0.96875 }, { 1, 1, 1 } };

float dgc_colormap_autumn[128][3] = { { 1, 0, 0 }, { 1, 0.007874, 0 }, { 1, 0.015748, 0 }, { 1, 0.023622, 0 }, { 1, 0.031496, 0 }, { 1, 0.03937, 0 }, { 1,
    0.047244, 0 }, { 1, 0.055118, 0 }, { 1, 0.062992, 0 }, { 1, 0.070866, 0 }, { 1, 0.07874, 0 }, { 1, 0.086614, 0 }, { 1, 0.094488, 0 }, { 1, 0.10236, 0 }, {
    1, 0.11024, 0 }, { 1, 0.11811, 0 }, { 1, 0.12598, 0 }, { 1, 0.13386, 0 }, { 1, 0.14173, 0 }, { 1, 0.14961, 0 }, { 1, 0.15748, 0 }, { 1, 0.16535, 0 }, { 1,
    0.17323, 0 }, { 1, 0.1811, 0 }, { 1, 0.18898, 0 }, { 1, 0.19685, 0 }, { 1, 0.20472, 0 }, { 1, 0.2126, 0 }, { 1, 0.22047, 0 }, { 1, 0.22835, 0 }, { 1,
    0.23622, 0 }, { 1, 0.24409, 0 }, { 1, 0.25197, 0 }, { 1, 0.25984, 0 }, { 1, 0.26772, 0 }, { 1, 0.27559, 0 }, { 1, 0.28346, 0 }, { 1, 0.29134, 0 }, { 1,
    0.29921, 0 }, { 1, 0.30709, 0 }, { 1, 0.31496, 0 }, { 1, 0.32283, 0 }, { 1, 0.33071, 0 }, { 1, 0.33858, 0 }, { 1, 0.34646, 0 }, { 1, 0.35433, 0 }, { 1,
    0.3622, 0 }, { 1, 0.37008, 0 }, { 1, 0.37795, 0 }, { 1, 0.38583, 0 }, { 1, 0.3937, 0 }, { 1, 0.40157, 0 }, { 1, 0.40945, 0 }, { 1, 0.41732, 0 }, { 1,
    0.4252, 0 }, { 1, 0.43307, 0 }, { 1, 0.44094, 0 }, { 1, 0.44882, 0 }, { 1, 0.45669, 0 }, { 1, 0.46457, 0 }, { 1, 0.47244, 0 }, { 1, 0.48031, 0 }, { 1,
    0.48819, 0 }, { 1, 0.49606, 0 }, { 1, 0.50394, 0 }, { 1, 0.51181, 0 }, { 1, 0.51969, 0 }, { 1, 0.52756, 0 }, { 1, 0.53543, 0 }, { 1, 0.54331, 0 }, { 1,
    0.55118, 0 }, { 1, 0.55906, 0 }, { 1, 0.56693, 0 }, { 1, 0.5748, 0 }, { 1, 0.58268, 0 }, { 1, 0.59055, 0 }, { 1, 0.59843, 0 }, { 1, 0.6063, 0 }, { 1,
    0.61417, 0 }, { 1, 0.62205, 0 }, { 1, 0.62992, 0 }, { 1, 0.6378, 0 }, { 1, 0.64567, 0 }, { 1, 0.65354, 0 }, { 1, 0.66142, 0 }, { 1, 0.66929, 0 }, { 1,
    0.67717, 0 }, { 1, 0.68504, 0 }, { 1, 0.69291, 0 }, { 1, 0.70079, 0 }, { 1, 0.70866, 0 }, { 1, 0.71654, 0 }, { 1, 0.72441, 0 }, { 1, 0.73228, 0 }, { 1,
    0.74016, 0 }, { 1, 0.74803, 0 }, { 1, 0.75591, 0 }, { 1, 0.76378, 0 }, { 1, 0.77165, 0 }, { 1, 0.77953, 0 }, { 1, 0.7874, 0 }, { 1, 0.79528, 0 }, { 1,
    0.80315, 0 }, { 1, 0.81102, 0 }, { 1, 0.8189, 0 }, { 1, 0.82677, 0 }, { 1, 0.83465, 0 }, { 1, 0.84252, 0 }, { 1, 0.85039, 0 }, { 1, 0.85827, 0 }, { 1,
    0.86614, 0 }, { 1, 0.87402, 0 }, { 1, 0.88189, 0 }, { 1, 0.88976, 0 }, { 1, 0.89764, 0 }, { 1, 0.90551, 0 }, { 1, 0.91339, 0 }, { 1, 0.92126, 0 }, { 1,
    0.92913, 0 }, { 1, 0.93701, 0 }, { 1, 0.94488, 0 }, { 1, 0.95276, 0 }, { 1, 0.96063, 0 }, { 1, 0.9685, 0 }, { 1, 0.97638, 0 }, { 1, 0.98425, 0 }, { 1,
    0.99213, 0 }, { 1, 1, 0 } };

float dgc_colormap_bone[128][3] = { { 0, 0, 0.0026042 }, { 0.0068898, 0.0068898, 0.012098 }, { 0.01378, 0.01378, 0.021592 }, { 0.020669, 0.020669, 0.031086 }, {
    0.027559, 0.027559, 0.04058 }, { 0.034449, 0.034449, 0.050074 }, { 0.041339, 0.041339, 0.059568 }, { 0.048228, 0.048228, 0.069062 }, { 0.055118, 0.055118,
    0.078556 }, { 0.062008, 0.062008, 0.08805 }, { 0.068898, 0.068898, 0.097543 }, { 0.075787, 0.075787, 0.10704 }, { 0.082677, 0.082677, 0.11653 }, { 0.089567,
    0.089567, 0.12603 }, { 0.096457, 0.096457, 0.13552 }, { 0.10335, 0.10335, 0.14501 }, { 0.11024, 0.11024, 0.15451 }, { 0.11713, 0.11713, 0.164 }, { 0.12402,
    0.12402, 0.17349 }, { 0.13091, 0.13091, 0.18299 }, { 0.1378, 0.1378, 0.19248 }, { 0.14469, 0.14469, 0.20198 }, { 0.15157, 0.15157, 0.21147 }, { 0.15846,
    0.15846, 0.22096 }, { 0.16535, 0.16535, 0.23046 }, { 0.17224, 0.17224, 0.23995 }, { 0.17913, 0.17913, 0.24945 }, { 0.18602, 0.18602, 0.25894 }, { 0.19291,
    0.19291, 0.26843 }, { 0.1998, 0.1998, 0.27793 }, { 0.20669, 0.20669, 0.28742 }, { 0.21358, 0.21358, 0.29692 }, { 0.22047, 0.22047, 0.30641 }, { 0.22736,
    0.22736, 0.3159 }, { 0.23425, 0.23425, 0.3254 }, { 0.24114, 0.24114, 0.33489 }, { 0.24803, 0.24803, 0.34439 }, { 0.25492, 0.25492, 0.35388 }, { 0.26181,
    0.26181, 0.36337 }, { 0.2687, 0.2687, 0.37287 }, { 0.27559, 0.27559, 0.38236 }, { 0.28248, 0.28248, 0.39186 }, { 0.28937, 0.28937, 0.40135 }, { 0.29626,
    0.29626, 0.41084 }, { 0.30315, 0.30315, 0.42034 }, { 0.31004, 0.31004, 0.42983 }, { 0.31693, 0.31693, 0.43932 }, { 0.32382, 0.32382, 0.44882 }, { 0.33071,
    0.33331, 0.45571 }, { 0.3376, 0.34281, 0.4626 }, { 0.34449, 0.3523, 0.46949 }, { 0.35138, 0.36179, 0.47638 }, { 0.35827, 0.37129, 0.48327 }, { 0.36516,
    0.38078, 0.49016 }, { 0.37205, 0.39028, 0.49705 }, { 0.37894, 0.39977, 0.50394 }, { 0.38583, 0.40926, 0.51083 }, { 0.39272, 0.41876, 0.51772 }, { 0.39961,
    0.42825, 0.52461 }, { 0.4065, 0.43775, 0.5315 }, { 0.41339, 0.44724, 0.53839 }, { 0.42028, 0.45673, 0.54528 }, { 0.42717, 0.46623, 0.55217 }, { 0.43406,
    0.47572, 0.55906 }, { 0.44094, 0.48522, 0.56594 }, { 0.44783, 0.49471, 0.57283 }, { 0.45472, 0.5042, 0.57972 }, { 0.46161, 0.5137, 0.58661 }, { 0.4685,
    0.52319, 0.5935 }, { 0.47539, 0.53269, 0.60039 }, { 0.48228, 0.54218, 0.60728 }, { 0.48917, 0.55167, 0.61417 }, { 0.49606, 0.56117, 0.62106 }, { 0.50295,
    0.57066, 0.62795 }, { 0.50984, 0.58016, 0.63484 }, { 0.51673, 0.58965, 0.64173 }, { 0.52362, 0.59914, 0.64862 }, { 0.53051, 0.60864, 0.65551 }, { 0.5374,
    0.61813, 0.6624 }, { 0.54429, 0.62762, 0.66929 }, { 0.55118, 0.63712, 0.67618 }, { 0.55807, 0.64661, 0.68307 }, { 0.56496, 0.65611, 0.68996 }, { 0.57185,
    0.6656, 0.69685 }, { 0.57874, 0.67509, 0.70374 }, { 0.58563, 0.68459, 0.71063 }, { 0.59252, 0.69408, 0.71752 }, { 0.59941, 0.70358, 0.72441 }, { 0.6063,
    0.71307, 0.7313 }, { 0.61319, 0.72256, 0.73819 }, { 0.62008, 0.73206, 0.74508 }, { 0.62697, 0.74155, 0.75197 }, { 0.63386, 0.75105, 0.75886 }, { 0.64075,
    0.76054, 0.76575 }, { 0.64764, 0.77003, 0.77264 }, { 0.65453, 0.77953, 0.77953 }, { 0.66532, 0.78642, 0.78642 }, { 0.67612, 0.79331, 0.79331 }, { 0.68692,
    0.8002, 0.8002 }, { 0.69771, 0.80709, 0.80709 }, { 0.70851, 0.81398, 0.81398 }, { 0.7193, 0.82087, 0.82087 }, { 0.7301, 0.82776, 0.82776 }, { 0.7409,
    0.83465, 0.83465 }, { 0.75169, 0.84154, 0.84154 }, { 0.76249, 0.84843, 0.84843 }, { 0.77328, 0.85531, 0.85531 }, { 0.78408, 0.8622, 0.8622 }, { 0.79488,
    0.86909, 0.86909 }, { 0.80567, 0.87598, 0.87598 }, { 0.81647, 0.88287, 0.88287 }, { 0.82726, 0.88976, 0.88976 }, { 0.83806, 0.89665, 0.89665 }, { 0.84886,
    0.90354, 0.90354 }, { 0.85965, 0.91043, 0.91043 }, { 0.87045, 0.91732, 0.91732 }, { 0.88124, 0.92421, 0.92421 }, { 0.89204, 0.9311, 0.9311 }, { 0.90284,
    0.93799, 0.93799 }, { 0.91363, 0.94488, 0.94488 }, { 0.92443, 0.95177, 0.95177 }, { 0.93522, 0.95866, 0.95866 }, { 0.94602, 0.96555, 0.96555 }, { 0.95682,
    0.97244, 0.97244 }, { 0.96761, 0.97933, 0.97933 }, { 0.97841, 0.98622, 0.98622 }, { 0.9892, 0.99311, 0.99311 }, { 1, 1, 1 } };

float dgc_colormap_jet[128][3] = { { 0, 0, 0.53125 }, { 0, 0, 0.5625 }, { 0, 0, 0.59375 }, { 0, 0, 0.625 }, { 0, 0, 0.65625 }, { 0, 0, 0.6875 },
    { 0, 0, 0.71875 }, { 0, 0, 0.75 }, { 0, 0, 0.78125 }, { 0, 0, 0.8125 }, { 0, 0, 0.84375 }, { 0, 0, 0.875 }, { 0, 0, 0.90625 }, { 0, 0, 0.9375 }, { 0, 0,
        0.96875 }, { 0, 0, 1 }, { 0, 0.03125, 1 }, { 0, 0.0625, 1 }, { 0, 0.09375, 1 }, { 0, 0.125, 1 }, { 0, 0.15625, 1 }, { 0, 0.1875, 1 }, { 0, 0.21875, 1 },
    { 0, 0.25, 1 }, { 0, 0.28125, 1 }, { 0, 0.3125, 1 }, { 0, 0.34375, 1 }, { 0, 0.375, 1 }, { 0, 0.40625, 1 }, { 0, 0.4375, 1 }, { 0, 0.46875, 1 },
    { 0, 0.5, 1 }, { 0, 0.53125, 1 }, { 0, 0.5625, 1 }, { 0, 0.59375, 1 }, { 0, 0.625, 1 }, { 0, 0.65625, 1 }, { 0, 0.6875, 1 }, { 0, 0.71875, 1 },
    { 0, 0.75, 1 }, { 0, 0.78125, 1 }, { 0, 0.8125, 1 }, { 0, 0.84375, 1 }, { 0, 0.875, 1 }, { 0, 0.90625, 1 }, { 0, 0.9375, 1 }, { 0, 0.96875, 1 },
    { 0, 1, 1 }, { 0.03125, 1, 0.96875 }, { 0.0625, 1, 0.9375 }, { 0.09375, 1, 0.90625 }, { 0.125, 1, 0.875 }, { 0.15625, 1, 0.84375 }, { 0.1875, 1, 0.8125 }, {
        0.21875, 1, 0.78125 }, { 0.25, 1, 0.75 }, { 0.28125, 1, 0.71875 }, { 0.3125, 1, 0.6875 }, { 0.34375, 1, 0.65625 }, { 0.375, 1, 0.625 }, { 0.40625, 1,
        0.59375 }, { 0.4375, 1, 0.5625 }, { 0.46875, 1, 0.53125 }, { 0.5, 1, 0.5 }, { 0.53125, 1, 0.46875 }, { 0.5625, 1, 0.4375 }, { 0.59375, 1, 0.40625 }, {
        0.625, 1, 0.375 }, { 0.65625, 1, 0.34375 }, { 0.6875, 1, 0.3125 }, { 0.71875, 1, 0.28125 }, { 0.75, 1, 0.25 }, { 0.78125, 1, 0.21875 }, { 0.8125, 1,
        0.1875 }, { 0.84375, 1, 0.15625 }, { 0.875, 1, 0.125 }, { 0.90625, 1, 0.09375 }, { 0.9375, 1, 0.0625 }, { 0.96875, 1, 0.03125 }, { 1, 1, 0 }, { 1,
        0.96875, 0 }, { 1, 0.9375, 0 }, { 1, 0.90625, 0 }, { 1, 0.875, 0 }, { 1, 0.84375, 0 }, { 1, 0.8125, 0 }, { 1, 0.78125, 0 }, { 1, 0.75, 0 }, { 1,
        0.71875, 0 }, { 1, 0.6875, 0 }, { 1, 0.65625, 0 }, { 1, 0.625, 0 }, { 1, 0.59375, 0 }, { 1, 0.5625, 0 }, { 1, 0.53125, 0 }, { 1, 0.5, 0 }, { 1, 0.46875,
        0 }, { 1, 0.4375, 0 }, { 1, 0.40625, 0 }, { 1, 0.375, 0 }, { 1, 0.34375, 0 }, { 1, 0.3125, 0 }, { 1, 0.28125, 0 }, { 1, 0.25, 0 }, { 1, 0.21875, 0 }, {
        1, 0.1875, 0 }, { 1, 0.15625, 0 }, { 1, 0.125, 0 }, { 1, 0.09375, 0 }, { 1, 0.0625, 0 }, { 1, 0.03125, 0 }, { 1, 0, 0 }, { 0.96875, 0, 0 }, { 0.9375, 0,
        0 }, { 0.90625, 0, 0 }, { 0.875, 0, 0 }, { 0.84375, 0, 0 }, { 0.8125, 0, 0 }, { 0.78125, 0, 0 }, { 0.75, 0, 0 }, { 0.71875, 0, 0 }, { 0.6875, 0, 0 }, {
        0.65625, 0, 0 }, { 0.625, 0, 0 }, { 0.59375, 0, 0 }, { 0.5625, 0, 0 }, { 0.53125, 0, 0 }, { 0.5, 0, 0 } };

float dgc_colormap_prism[128][3] = { { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 },
    { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 },
    { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
    { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 },
    { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 },
    { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
    { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 },
    { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 },
    { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
    { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 },
    { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 },
    { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
    { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.66667, 0, 1 }, { 1, 0, 0 }, { 1, 0.5, 0 } };

} // namespace glsupport
