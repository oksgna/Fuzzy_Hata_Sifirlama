#include <stdio.h>
#include <stdlib.h>
#include <math.h>
 
// --- Sabitler ---
#define STATE_SIZE 2 // Durum vektörünün boyutu: [Akım (i), Hız (w)]
 
// --- DC Motor Parametreleri (Python kodundaki değerler kullanıldı) ---
const double R = 1.0;
const double L = 0.5;
const double Kt = 0.01;
const double Kb = 0.01;
const double J = 0.01;
const double B = 0.001;
 
// x: Mevcut durum [i, w]
// u: Kontrol Gerilimi
// TL: Yük Torku (Varsayılan 0.0)
// dx_dt: Türevleri döndüren çıkış dizisi [di/dt, dw/dt]
 
void motor_derivatives(const double x[], double u, double TL, double dx_dt[]) {
    double i = x[0]; // Akım
    double w = x[1]; // Açısal Hız
    
    dx_dt[0] = (-R * i - Kb * w + u) / L;
   
    dx_dt[1] = (-B * w + Kt * i - TL) / J;
}
 
// x_current: Mevcut durum [i, w]
// u: Kontrol Gerilimi
// dt: Zaman adımı
// TL: Yük Torku (Varsayılan 0.0)
// x_next: Yeni durumu döndüren çıkış dizisi [i_yeni, w_yeni]
 
void rk4_step(const double x_current[], double u, double dt, double TL, double x_next[]) {
    
    // Geçici katsayı ve durum dizileri
    double k1[STATE_SIZE], k2[STATE_SIZE], k3[STATE_SIZE], k4[STATE_SIZE];
    double x_temp[STATE_SIZE]; 
    
    int j;
    
    // --- k1 = motor_derivatives(x, u, TL) ---
    motor_derivatives(x_current, u, TL, k1);
    
    // --- k2 = motor_derivatives(x + 0.5*dt*k1, u, TL) ---
    for (j = 0; j < STATE_SIZE; j++) {
        x_temp[j] = x_current[j] + 0.5 * dt * k1[j];
    }
    motor_derivatives(x_temp, u, TL, k2);
    
    // --- k3 = motor_derivatives(x + 0.5*dt*k2, u, TL) ---
    for (j = 0; j < STATE_SIZE; j++) {
        x_temp[j] = x_current[j] + 0.5 * dt * k2[j];
    }
    motor_derivatives(x_temp, u, TL, k3);
    
    // --- k4 = motor_derivatives(x + dt*k3, u, TL) ---
    for (j = 0; j < STATE_SIZE; j++) {
        x_temp[j] = x_current[j] + dt * k3[j];
    }
    motor_derivatives(x_temp, u, TL, k4);
    
    // --- return x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4) ---
    for (j = 0; j < STATE_SIZE; j++) {
        x_next[j] = x_current[j] + (dt / 6.0) * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
    }
}

