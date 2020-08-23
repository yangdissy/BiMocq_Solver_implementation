#ifndef _BiMocq_
#define _BiMocq_

// struct Global {
// 	int velReinit = 0;
//     int scalarReinit = 0;
// 	float volumesize = 0.1;
// 	float substep = 0.04666;
// 	vector test = set(1.0, 1.0, 1.0);
// }
vector semilagAdvectVelocity(int _VvolumeN;string _Vvolumename;int _Velnumber;string _Velname;vector _Pos;float _substep){
	vector pos =_Pos;
	float substep = _substep;
	vector vel = volumesamplev(_Velnumber, _Velname, pos);
	vector finalpos = pos - vel * substep;
	vector advectvel = volumesamplev(_VvolumeN, _Vvolumename, finalpos);
	return advectvel;
}
float semilagAdvectField(int _VolumeN;string _Volumename;int _Velnumber;string _Velname;vector _Pos;float _substep){
	vector pos =_Pos;
	float substep = _substep;
	vector vel = volumesamplev(_Velnumber, _Velname, pos);
	vector finalpos = pos - vel * substep;
	float advectfield = volumesample(_VvolumeN, _Vvolumename, finalpos);
	return advectfield;
}

vector volumegradientv(int _VolumeN;string _Volumename;vector _Pos){
	// vector Du = volumegradient(_VolumeN,_Volumename +'.x',_Pos);
	// vector Dv = volumegradient(_VolumeN,_Volumename +'.y',_Pos);
	// vector Dw = volumegradient(_VolumeN,_Volumename +'.z',_Pos);

	// return Du+Dv+Dw;
	vector pos = _Pos;
	int evaluations = 8;
	float weight = 1.0/float(evaluations);
	vector sum = set(0, 0, 0);
	vector index = volumepostoindex(_VolumeN, _Volumename +'.x', pos);
	vector offset[];
	offset[0] = set(index.x+1, index.y+1, index.z+1);
	offset[1] = set(index.x+1, index.y+1, index.z-1);
	offset[2] = set(index.x+1, index.y-1, index.z+1);
	offset[3] = set(index.x+1, index.y-1, index.z-1);
	offset[4] = set(index.x-1, index.y+1, index.z+1);
	offset[5] = set(index.x-1, index.y+1, index.z-1);
	offset[6] = set(index.x-1, index.y-1, index.z+1);
	offset[7] = set(index.x-1, index.y-1, index.z-1);

	for (int i = 0; i < evaluations; ++i){
		/* code */
		vector voxel_pos = volumeindextopos(_VolumeN, _Volumename +'.x',offset[i]);
		vector Dvel = weight * volumesamplev(_VolumeN, _Volumename, voxel_pos);
		sum = sum + Dvel;
	}
	return sum;

}
float volumegradient(int _VolumeN;string _Volumename;vector _Pos){
	// vector Du = volumegradient(_VolumeN,_Volumename +'.x',_Pos);
	// vector Dv = volumegradient(_VolumeN,_Volumename +'.y',_Pos);
	// vector Dw = volumegradient(_VolumeN,_Volumename +'.z',_Pos);

	// return Du+Dv+Dw;
	vector pos = _Pos;
	int evaluations = 8;
	float weight = 1.0/float(evaluations);
	float sum = 0;
	vector index = volumepostoindex(_VolumeN, _Volumename, pos);
	vector offset[];
	offset[0] = set(index.x+1, index.y+1, index.z+1);
	offset[1] = set(index.x+1, index.y+1, index.z-1);
	offset[2] = set(index.x+1, index.y-1, index.z+1);
	offset[3] = set(index.x+1, index.y-1, index.z-1);
	offset[4] = set(index.x-1, index.y+1, index.z+1);
	offset[5] = set(index.x-1, index.y+1, index.z-1);
	offset[6] = set(index.x-1, index.y-1, index.z+1);
	offset[7] = set(index.x-1, index.y-1, index.z-1);

	for (int i = 0; i < evaluations; ++i){
		/* code */
		vector voxel_pos = volumeindextopos(_VolumeN, _Volumename,offset[i]);
		float Dvel = weight * volumesample(_VolumeN, _Volumename, voxel_pos);
		sum = sum + Dvel;
	}
	return sum;

}
vector velocityDelta(int _VolumeN;string _Volumename;vector _Pos){
	vector Du = volumegradient(_VolumeN,_Volumename +'.x',_Pos);
	vector Dv = volumegradient(_VolumeN,_Volumename +'.y',_Pos);
	vector Dw = volumegradient(_VolumeN,_Volumename +'.z',_Pos);
	vector Delta = Du + Dv + Dw;
	return Delta;

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
	
	float new_x = (abs(a_x) != 0)? pos.x + (1 - exp(-a_x*substep))*vel.x/a_x : pos.x + vel.x*substep;
	float new_y = (abs(a_y) != 0)? pos.y + (1 - exp(-a_y*substep))*vel.y/a_y : pos.y + vel.y*substep;
	float new_z = (abs(a_z) != 0)? pos.z + (1 - exp(-a_z*substep))*vel.z/a_z : pos.z + vel.z*substep;
	vector pointnew = set(new_x, new_y, new_z);

	vector output = volumesamplev(_Velnumber,_Velname,pointnew);

	return output;
}
vector GetVectorByRK3(int _Velnumber;string _Velname;vector _Pos;float _substep){
	float substep = _substep;
	vector pos = _Pos;
	float c1 = 2.0/9.0, c2 = 3.0/9.0, c3 = 4.0/9.0;
	vector v1 = volumesamplev(_Velnumber, _Velname, pos);
	vector midp1 = set(pos.x-0.5*substep*v1.x, pos.y-0.5*substep*v1.y, pos.z-0.5*substep*v1.z);
	vector v2 = volumesamplev(_Velnumber, _Velname, midp1);
	vector midp2 = set(pos.x-0.75*substep*v2.x, pos.y-0.75*substep*v2.y, pos.z-0.75*substep*v2.z);
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
	vector Vgradent = volumegradientv(_Velinitnumber, _Velinitname, finalpos);
	vector BackwardVel = volumesamplev(_Velinitnumber, _Velinitname, finalpos);
	vector Value = 0.5 * BackwardVel + 0.5 * Vgradent;
	return Value; 
}
float AdvectField(int _Fieldinitnumber;string _Fieldinitname;int _backwardvN;string _backwardvname;float _substep;vector _Pos;){
	vector pos = _Pos;
	vector backwardv = volumesamplev(_backwardvN, _backwardvname, pos);
	vector finalpos = pos - backwardv * _substep;
	float  Vgradent = volumegradient(_Fieldinitnumber, _Fieldinitname, finalpos);
	vector BackwardVel = volumesample(_Fieldinitnumber, _Fieldinitname, finalpos);
	float  Value = 0.5 * BackwardVel + 0.5 * Vgradent;
	return Value; 
}
vector cumulateVelocity(int _VolumeN;string _Volumename;int _VolumeNTemp;string _VolumenameTemp;int _backwardvN;string _backwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	float coeff = 1.0;
	
	vector backwardv = volumesamplev(_backwardvN,_backwardvname,pos);
	vector finalpos = pos - backwardv * _substep;
	vector dveltemp = coeff * volumegradientv(_VolumeNTemp, _VolumenameTemp, finalpos);
	vector veltemp = coeff * volumesamplev(_VolumeNTemp, _VolumenameTemp, finalpos);
	vector sum =  0.5 * veltemp + 0.5 * dveltemp;
	vector value = volumesamplev(_VolumeN,_Volumename,finalpos);
	vector cumulate = value + sum;
	return cumulate;
}
float cumulateField(int _FieldN;string _Fieldname;int _FieldNTemp;string _FieldnameTemp;int _backwardvN;string _backwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	float coeff = 1.0;
	
	vector backwardv = volumesamplev(_backwardvN,_backwardvname,pos);
	vector finalpos = pos - backwardv * _substep;
	float dfieldtemp = coeff * volumegradient(_FieldNTemp, _FieldnameTemp, finalpos);
	float fieldtemp = coeff * volumesample(_FieldNTemp, _FieldnameTemp, finalpos);
	float sum =  0.5 * fieldtemp + 0.5 * dfieldtemp;
	float value = volumesample(_FieldN, _Fieldname, pos);
	float cumulate = value + sum;
	return sum;
}
vector compensateVelocity(int _Velnumber;string _Velname;int _Velinitnumber;string _Velinitname;int _forwardvN;string _forwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	vector forwardv = volumesamplev(_forwardvN, _forwardvname, pos);
	vector finalpos = pos + forwardv * _substep;
	//vector dforward = volumegradient(_forwardvN, _forwardvname, pos);
	//vector forwardpos = forwardvel * _substep + pos;
	//vector forwardvel = volumesamplev(_forwardvN, _forwardvname, pos);
	vector currentvel = volumesamplev(_Velnumber,_Velname,finalpos);
	vector dcurrentvel = volumegradientv(_Velnumber,_Velname,finalpos);
	vector sum = 0.5*currentvel + 0.5*dcurrentvel;
	vector Vtemp = sum  - volumesamplev(_Velinitnumber, _Velinitname, pos);
	//vector Vtemp =  backwardprev - sum;
	return Vtemp;
}
float compensatefield(int _Fieldnumber;string _Fieldname;int _Fieldinitnumber;string _Fieldinitname;int _forwardvN;string _forwardvname;float _substep;vector _Pos){
	vector pos = _Pos;
	vector forwardv = volumesamplev(_forwardvN, _forwardvname, pos);
	vector finalpos = pos + forwardv * _substep;
	float currentfield = volumesample(_Fieldnumber,_Fieldname,finalpos);
	float dcurrentdield = volumegradient(_Fieldnumber,_Fieldname,finalpos);
	float sum = 0.5*currentfield + 0.5*dcurrentdield;
	float Ftemp = sum - volumesample(_Fieldinitnumber, _Fieldinitname, pos);
	return Ftemp;
}
vector DoubleAdvectVelocity(int _Velnumber;string _Velname;int _Veltempnumber;string _Veltempname;int _backwardPrevN;string __backwardPrevname ;int _backwardvN;string _backwardvname;float _substep;vector _Pos;float _blend_coeff){
	
	vector pos = _Pos;
	float blend_coeff = _blend_coeff;

	//vector currentvel = volumesamplev(_Velnumber, _Velname, pos); 
	//vector backwardpos = pos - currentvel*_substep;
	//X1(xд)
	vector backwardv = volumesample(_backwardvN, _backwardvname, pos);
	vector midpos = pos - backwardv * _substep;
	//x = Xt (xд)
	vector backwardprev = volumesamplev(_backwardPrevN, __backwardPrevname, midpos);	
	vector finalpos = midpos - backwardprev * _substep;
	vector vel = volumesamplev(_Velnumber, _Velname,pos);
	vector velprev = volumesamplev(_Velnumber, _Velname,finalpos);
	vector velcurrent = volumesamplev(_Velnumber, _Velname,midpos);
	vector dvelprev = volumesamplev(_Veltempnumber,_Veltempname,finalpos);
	vector dvelcurrent = volumesamplev(_Veltempnumber,_Veltempname,midpos);
	//vector dvelprev = _substep * velocityDelta(_Velnumber, _Velname,finalpos);
	//vector dvelcurrent = _substep * velocityDelta(_Velnumber, _Velname,midpos);
	//vector Deltaprev = volumesamplev(_Veltempnumber,_Veltempname,finalpos);
	//vector Deltacurrent = volumesamplev(_Veltempnumber,_Veltempname,midpos);
	//0.5*velprev + 0.5*velcurrent + 0.5*dvelprev + dvelcurrent
	//+ 0.5*dvelprev + dvelcurrent
 
	vector prev_value = vel - 0.5*(velprev + velcurrent ) ;

	return prev_value;
	
}
float DoubleAdvectField(int _Fieldnumber;string _Fieldname;int _FieldTempnumber;string _FieldTempname;int _backwardPrevN;string __backwardPrevname ;int _backwardvN;string _backwardvname;float _substep;vector _Pos;float _blend_coeff){
	
	vector pos = _Pos;
	float blend_coeff = _blend_coeff;

	//vector currentvel = volumesamplev(_Velnumber, _Velname, pos); 
	//vector backwardpos = pos - currentvel*_substep;
	//X1(xд)
	vector backwardv = volumesample(_backwardvN, _backwardvname, pos);
	vector midpos = pos - backwardv * _substep;
	//x = Xt (xд)
	vector backwardprev = volumesamplev(_backwardPrevN, __backwardPrevname, midpos);	
	vector finalpos = midpos - backwardprev * _substep;
	//float dfieldtemp = volumegradient(_Fieldnumber, _Fieldname, finalpos);
	//float fieldtemp = volumesample(_Fieldnumber, _Fieldname, midpos);
	float fieldprev = volumesample(_Fieldnumber, _Fieldname,finalpos);
	float fieldcurrent = volumesample(_Fieldnumber, _Fieldname,midpos);
	float dfieldprev =  volumesample(_FieldTempnumber,_FieldTempname,finalpos);
	float dfieldcurrent =  volumesample(_FieldTempnumber,_FieldTempname,midpos);
	float prev_value = 0.5*fieldprev + fieldcurrent;
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
	vector back_Pos = pos + backward * substep;
	vector forwardBF = volumesamplev(_forwardvN, _forwardvname, back_Pos);
	vector Fwd_Pos = back_Pos + forwardBF * substep;
	float dist_bf = distance(pos,Fwd_Pos);
	//forward then backward
	vector forward = volumesamplev(_forwardvN, _forwardvname, pos);
	vector fwd_Pos = pos + forward * substep;
	vector backwardFB = volumesamplev(_backwardvN, _backwardvname, fwd_Pos); 
	vector Back_Pos = fwd_Pos + backwardFB * substep;
	float dist_fb = distance(pos,Back_Pos);

	return VDistortion = max(dist_bf, dist_fb);
}
float estimateFieldDistortion(int _VolumeN;string _Volumename;int _backwardvN;string _backwardvname;int _forwardvN;string _forwardvname;vector _Pos;float _substep){
	vector pos = _Pos;
	float FDistortion;
	float substep = _substep;
	//backward then forward
	vector backward = volumesamplev(_backwardvN, _backwardvname, pos); 
	vector back_Pos = pos + backward * substep;
	vector forwardBF = volumesamplev(_forwardvN, _forwardvname, back_Pos);
	vector Fwd_Pos = back_Pos + forwardBF * substep;
	float distBF = volumesample(_VolumeN, _Volumename, Fwd_Pos);
	//forward then backward
	vector forward = volumesamplev(_forwardvN, _forwardvname, pos);
	vector fwd_Pos = pos + forward * substep;
	vector backwardFB = volumesamplev(_backwardvN, _backwardvname, fwd_Pos); 
	vector Back_Pos = fwd_Pos + backwardFB * substep;
	float distFB = volumesample(_VolumeN, _Volumename, Back_Pos);

	return FDistortion = max(distBF, distFB);
}
vector accumulateVelocity(int _VelchangeN;string _Velchangename;int _VelinitN;string _Velinitname;int _forwardvN;string _forwardvname;float _substep;vector _Pos;float _coeff){
	vector pos = _Pos;
	float coeff = _coeff;
	vector forwardv = coeff * volumesamplev(_forwardvN, _forwardvname, pos);
	vector map_pos = pos + forwardv * _substep;
	vector dvel = coeff * volumegradientv(_VelchangeN, _Velchangename, map_pos);
	vector velchange = volumesamplev(_VelchangeN, _Velchangename, map_pos);
	vector sum = 0.5 * dvel + 0.5 * velchange;
	vector dvelcity_init = dvelcity_init + sum;
	return dvelcity_init;
}
float accumulateField(int _FieldchangeN;string _Fieldchangename;int _Fieldinitnumber;string _Fieldinitname;int _forwardvN;string _forwardvname;float _substep;vector _Pos;float _coeff){
	vector pos = _Pos;
	float 	coeff = _coeff;
	vector forwardv = coeff * volumesamplev(_forwardvN, _forwardvname, pos);
	vector map_pos = pos + forwardv * _substep;
	float dvel = coeff * volumegradient(_FieldchangeN, _Fieldchangename, map_pos);
	float velchange = volumesample(_FieldchangeN, _Fieldchangename, pos);
	float sum = 0.5 * dvel + 0.5 * velchange;
	float dvelcity_init = dvelcity_init + sum;
	return dvelcity_init;
}

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