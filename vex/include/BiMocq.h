#ifndef _BiMocq_
#define _BiMocq_

struct BimocqGlobal{
	int velReinit = 0;
    int scalarReinit = 0;
	float volumesize = 0.1;
	float substep = 0.04666;
}


vector AdvectVectorDMC(int _Velnumber;string _Velname; vector _Pos;float _volumesize;float _substep){

	float h = _volumesize;
	vector pos = _Pos;
	float substep = _substep;
	vector vel = volumesamplev(_Velnumber,_Velname, pos);
	float temp_x = (vel.x > 0)? pos.x - h: pos.x + h;
	float temp_y = (vel.y > 0)? pos.y - h: pos.y + h;
	float temp_z = (vel.z > 0)? pos.z - h: pos.z + h;
	vector temp_point = set(temp_x, temp_y, temp_z);
	vector temp_vel = volumesamplev(_Velnumber,_Velname,temp_point);
	float a_x = (vel.x - temp_vel.x) / (pos.x - temp_point.x);
	float a_y = (vel.y - temp_vel.y) / (pos.y - temp_point.y);
	float a_z = (vel.z - temp_vel.z) / (pos.z - temp_point.z);
	
	float new_x = (abs(a_x) != 0)? pos.x - (1 - exp(-a_x*substep))*vel.x/a_x : pos.x - vel.x*substep;
	float new_y = (abs(a_y) != 0)? pos.y - (1 - exp(-a_y*substep))*vel.y/a_y : pos.y - vel.y*substep;
	float new_z = (abs(a_z) != 0)? pos.z - (1 - exp(-a_z*substep))*vel.z/a_z : pos.z - vel.z*substep;
	vector pointnew = set(new_x, new_y, new_z);

	vector output = volumesamplev(_Velnumber,_Velname,pointnew);

	return output;
}
float AdvectFieldDMC(int _Velnumber;string _Velname; vector _Pos;float _volumesize;float _substep){

	float h = _volumesize;
	vector pos = _Pos;
	float substep = _substep;
	vector vel = volumesamplev(_Velnumber, _Velname, pos);
	float temp_x = (vel.x > 0)? pos.x - h: pos.x + h;
	float temp_y = (vel.y > 0)? pos.y - h: pos.y + h;
	float temp_z = (vel.z > 0)? pos.z - h: pos.z + h;
	vector temp_point = set(temp_x, temp_y, temp_z);
	vector temp_vel = volumesamplev(_Velnumber, _Velname, temp_point);
	float a_x = (vel.x - temp_vel.x) / (pos.x - temp_point.x);
	float a_y = (vel.y - temp_vel.y) / (pos.y - temp_point.y);
	float a_z = (vel.z - temp_vel.z) / (pos.z - temp_point.z);
	
	float new_x = (abs(a_x) > 1e-4)? pos.x - (1 - exp(-a_x*substep))*vel.x/a_x : pos.x - vel.x*substep;
	float new_y = (abs(a_y) > 1e-4)? pos.y - (1 - exp(-a_y*substep))*vel.y/a_y : pos.y - vel.y*substep;
	float new_z = (abs(a_z) > 1e-4)? pos.z - (1 - exp(-a_z*substep))*vel.z/a_z : pos.z - vel.z*substep;
	vector pointnew = set(new_x, new_y, new_z);

	float output = volumesample(_Velnumber, _Velname,pointnew);

	return output;
}
float GetFieldByRK3(int _VolumeN;string _Volumename;int _Velnumber;string _Velname;vector _Pos;float _substep){
	float c1 = 2.0/9.0, c2 = 3.0/9.0, c3 = 4.0/9.0;
	vector pos = _Pos;
	float f1 = volumesample(_VolumeN, _Volumename, pos);
	vector v1 = volumesamplev(_Velnumber, _Velname, pos);
	vector midp1 = set(pos.x+0.5*substep*v1.x, pos.y+0.5*substep*v1.y, pos.z+0.5*substep*v1.z);
	vector v2 = volumesamplev(_Velnumber, _Velname, midp1);
	float f2 = volumesample(_VolumeN, _Volumename, midp1);
	vector midp2 = set(pos.x+0.75*substep*v2.x, pos.y+0.75*substep*v2.y, pos.z+0.75*substep*v2.z);
	float f3 = volumesample(_VolumeN, _Volumename, midp2);
	float output = c1*f1 + c2*f2 + c3*f3;
	return output;
}
vector GetVectorByRK3(int _Velnumber;string _Velname;vector _Pos;float _substep){
	float substep = _substep;
	vector pos = _Pos;
	float c1 = 2.0/9.0, c2 = 3.0/9.0, c3 = 4.0/9.0;
	vector v1 = volumesamplev(_Velnumber, _Velname, pos);
	vector midp1 = set(pos.x+0.5*substep*v1.x, pos.y+0.5*substep*v1.y, pos.z+0.5*substep*v1.z);
	vector v2 = volumesamplev(_Velnumber, _Velname, midp1);
	vector midp2 = set(pos.x+0.75*substep*v2.x, pos.y+0.75*substep*v2.y, pos.z+0.75*substep*v2.z);
	vector v3 = volumesamplev(_Velnumber, _Velname, midp2);

	vector output = set(c1*v1.x + c2*v2.x + c3*v3.x,
                        c1*v1.y + c2*v2.y + c3*v3.y,
                        c1*v1.z + c2*v2.z + c3*v3.z);
	return output;
}
vector AdvectVelocity(int _Velinitnumber;string _Velinitname;int _backwardvN;string _backwardvname;float _substep;vector _Pos;){
	vector pos = _Pos;
	vector backwardv = volumesamplev(_backwardvN, _backwardvname, pos);
	vector finalpos = pos - backwardv * _substep;
	vector Vgradent = volumegradient(_Velinitnumber, _Velinitname, finalpos);
	vector BackwardVel = volumesamplev(_Velinitnumber, _Velinitname, finalpos);
	vector Value = 0.5 * Vgradent + 0.5 * BackwardVel;
	return Value; 
}
vector cumulateVelocity(int _VolumeN;string _Volumename;int _backwardvN;string _backwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	float coeff = 1.0;
	
	vector backwardv = volumesamplev(_backwardvN,_backwardvname,pos);
	vector finalpos = pos - backwardv * _substep;
	vector dvelinit = coeff * volumegradient(_VolumeN, _Volumename, finalpos);
	vector velinit = coeff * volumesamplev(_VolumeN, _Volumename, finalpos);
	vector sum =  0.5 * velinit + 0.5 * dvelinit;
	//vector dvelcity_init = dvelcity_init + sum;

	return sum;
}
vector compensateVelocity(int _Velnumber;string _Velname;int _forwardvN;string _forwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	vector forwardv = volumesamplev(_forwardvN, _forwardvname, pos);
	vector finalpos = pos + forwardv * _substep;
	//vector dforward = volumegradient(_forwardvN, _forwardvname, pos);
	//vector forwardpos = forwardvel * _substep + pos;
	//vector forwardvel = volumesamplev(_forwardvN, _forwardvname, pos);
	vector currentvel = volumesamplev(_Velnumber,_Velname,finalpos);
	vector dcurrentvel = volumegradient(_Velnumber,_Velname,finalpos);
	vector sum = 0.5*currentvel + 0.5*dcurrentvel;
	vector Vtemp = sum - volumegradient(_Velnumber, _Velname, pos);
	return Vtemp;
}
vector DoubleAdvectVelocity(int _Velnumber;string _Velname;int _backwardPrevN;string __backwardPrevname ;int _backwardvN;string _backwardvname;float _substep;vector _Pos;float _blend_coeff){
	vector pos = _Pos;
	float blend_coeff = _blend_coeff;
	vector backwardv = volumesample(_backwardvN, _backwardvname, pos);
	vector midpos = pos - backwardv * _substep;
	vector backwardprev = volumesamplev(_backwardPrevN, __backwardPrevname, midpos);
	vector finalpos = midpos - backwardprev * _substep;
	vector dveltemp = volumegradient(_Velnumber, _Velname, finalpos);
	vector veltemp = volumesample(_Velnumber, _Velname, finalpos);
	vector prev_value = 0.5 * (dveltemp + veltemp);
	//vector vel = volumesample(_Velnumber, _Velname, finalpos);
	//vel = vel * blend_coeff + (1.0-blend_coeff) * prev_value;

	return prev_value;
	
}

vector updateBackwardVectorDMC(int _Velnumber;string _Velname;vector _Pos;float _volumesize;float _substep){
    float h = _volumesize;
	vector pos = _Pos;
	float substep = _substep;
    //vector index = volumepostoindex(_Velnumber, _Velname,pos);
	//vector totalvox = volumeres(_Velname, _Velname);
	vector Backward = AdvectVectorDMC(_Velnumber,_Velname,pos,h,substep);
	return Backward;


}
vector updateForwardVectorRK3(int _Velnumber;string _Velname;vector _Pos;float _volumesize;float _substep){
    float h = _volumesize;
	vector pos = _Pos;
	float substep = _substep;
	vector Forward = GetVectorByRK3(_Velnumber,_Velname,pos,substep);

	return Forward;
}
float estimateVectorDistortion(int _backwardvN;string _backwardvname;int _forwardvN;string _forwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	float VDistortion;
	float substep = _substep;
	//backward then forward
	vector backward = volumesamplev(_backwardvN, _backwardvname, pos); 
	vector back_Pos = pos - backward * substep;
	vector forwardBF = volumesamplev(_forwardvN, _forwardvname, back_Pos);
	vector Fwd_Pos = back_Pos + forwardBF * substep;
	float dist_bf = distance(pos,Fwd_Pos);
	//forward then backward
	vector forward = volumesamplev(_forwardvN, _forwardvname, pos);
	vector fwd_Pos = pos + forward * substep;
	vector backwardFB = volumesamplev(_backwardvN, _backwardvname, fwd_Pos); 
	vector Back_Pos = fwd_Pos - backwardFB * substep;
	float dist_fb = distance(pos,Back_Pos);

	return VDistortion = max(dist_bf, dist_fb);
}
float estimateFieldDistortion(int _VolumeN;string _Volumename;int _backwardvN;string _backwardvname;int _forwardvN;string _forwardvname;vector _Pos;float _substep){
	vector pos = _Pos;
	float FDistortion;
	float substep = _substep;
	//backward then forward
	vector backward = volumesamplev(_backwardvN, _backwardvname, pos); 
	vector back_Pos = pos - backward * substep;
	vector forwardBF = volumesamplev(_forwardvN, _forwardvname, back_Pos);
	vector Fwd_Pos = back_Pos + forwardBF * substep;
	float distBF = volumesample(_VolumeN, _Volumename, Fwd_Pos);
	//forward then backward
	vector forward = volumesamplev(_forwardvN, _forwardvname, pos);
	vector fwd_Pos = pos + forward * substep;
	vector backwardFB = volumesamplev(_backwardvN, _backwardvname, fwd_Pos); 
	vector Back_Pos = fwd_Pos - backwardFB * substep;
	float distFB = volumesample(_VolumeN, _Volumename, Back_Pos);

	return FDistortion = max(distBF, distFB);
}
vector accumulateVelocity(int _VelchangeN;string _Velchangename;int _VelinitN;string _Velinitname;int _forwardvN;string _forwardvname;float _substep;vector _Pos;float _coeff){
	vector pos = _Pos;
	float coeff = _coeff;
	vector forwardv = coeff * volumegradient(_forwardvN, _forwardvname, pos);
	vector map_pos = pos + forwardv * _substep;
	vector dvel = coeff * volumegradient(_VelchangeN, _Velchangename, map_pos);
	vector velchange = volumesamplev(_VelchangeN, _Velchangename, pos);
	vector sum = 0.5 * dvel + 0.5 * velchange;
	vector dvelcity_init = dvelcity_init + sum;
	return dvelcity_init;
}
void accumulateField(int _VolumechangeN;string _Volumechangename;int _VolumeinitN;string _Volumeinitname;int _forwardvN;string _forwardvname;float _substep;vector _Pos;float _coeff){
	vector pos = _Pos;
	float coeff = 1.0f;
	vector forwardv = coeff * volumegradient(_forwardvN, _forwardvname, pos);
	vector map_pos = pos + forwardv * _substep;
	vector dvel = coeff * volumesample(_VolumechangeN, _Volumechangename, map_pos);
	vector velchange = volumesample(_VolumechangeN, _Volumechangename, pos);
	vector sum = 0.5 * dvel + 0.5 * velchange;
	vector dvelcity_init = dvelcity_init + sum;
	return dvelcity_init;
}
void ApplyChangeForwardMap(){}
void reinitializeMapping(){}

void advanceBimocq(){
	float proj_coeff = 2.f;
    int velReinit = 0;
    int scalarReinit = 0;
	float VelocityDistortion = estimateVectorDistortion();
	if (VelocityDistortion > 1.f || framenum - vel_lastReinit > 10){
		velReinit = true;
		vel_lastReinit = framenum;
		proj_coeff = 1.f;
	}
}



void printTestEscaping() {
	string a = "from myLib.h: \\n \\t v\@P, %04.2f";
	printf(a + "\n");
}

void myRemPoints(int ptnum) {
	if (ptnum < 30)
		removepoint(0, ptnum);
}




#endif // end of include guards