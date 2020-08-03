#ifndef _BiMocq_
#define _BiMocq_


//float cfldt = getCFL(); 

vector AdvectVectorDMC(int geometry,string volumename vector _pos,float _volumesize){

	float h = _volumesize;
	vector pos = _pos;
	vector vel = volumesamplev(geometry, 'vel', pos);


	float temp_x = (vel.x > 0)? pos.x - h: pos.x + h;
	float temp_y = (vel.y > 0)? pos.y - h: pos.y + h;
	float temp_z = (vel.z > 0)? pos.z - h: pos.z + h;
	vector temp_point = set(temp_x, temp_y, temp_z);
	vector temp_vel = volumesamplev(geometry, 'vel', temp_point);
	float a_x = (vel.x - temp_vel.x) / (pos.x - temp_point.x);
	float a_y = (vel.y - temp_vel.y) / (pos.y - temp_point.y);
	float a_z = (vel.z - temp_vel.z) / (pos.z - temp_point.z);
	
	float new_x = (abs(a_x) > 1e-4)? pos.x - (1 - exp(-a_x*substep))*vel.x/a_x : pos.x - vel.x*substep;
	float new_y = (abs(a_y) > 1e-4)? pos.y - (1 - exp(-a_y*substep))*vel.y/a_y : pos.y - vel.y*substep;
	float new_z = (abs(a_z) > 1e-4)? pos.z - (1 - exp(-a_z*substep))*vel.z/a_z : pos.z - vel.z*substep;
	vector pointnew = set(new_x, new_y, new_z);

	vector output = volumesamplev(geometry, volumename,pointnew);

	return output;
}
float AdvectFieldDMC(int geometry,string volumename vector _pos,float _volumesize){

	float h = _volumesize;
	vector pos = _pos;
	vector vel = volumesamplev(geometry, 'vel', pos)


	float temp_x = (vel.x > 0)? pos.x - h: pos.x + h;
	float temp_y = (vel.y > 0)? pos.y - h: pos.y + h;
	float temp_z = (vel.z > 0)? pos.z - h: pos.z + h;
	vector temp_point = set(temp_x, temp_y, temp_z);
	vector temp_vel = volumesamplev(geometry, 'vel', temp_point);
	float a_x = (vel.x - temp_vel.x) / (pos.x - temp_point.x);
	float a_y = (vel.y - temp_vel.y) / (pos.y - temp_point.y);
	float a_z = (vel.z - temp_vel.z) / (pos.z - temp_point.z);
	
	float new_x = (abs(a_x) > 1e-4)? pos.x - (1 - exp(-a_x*substep))*vel.x/a_x : pos.x - vel.x*substep;
	float new_y = (abs(a_y) > 1e-4)? pos.y - (1 - exp(-a_y*substep))*vel.y/a_y : pos.y - vel.y*substep;
	float new_z = (abs(a_z) > 1e-4)? pos.z - (1 - exp(-a_z*substep))*vel.z/a_z : pos.z - vel.z*substep;
	vector pointnew = set(new_x, new_y, new_z);

	float output = volumesample(geometry, volumename,pointnew);

	return output;
}
float GetFieldByRK3(int geometry,string volumename,vector pos,float subteps){
	float c1 = 2.0/9.0*subteps, c2 = 3.0/9.0 * subteps, c3 = 4.0/9.0 * subteps;
	vector input = pos;
	float f1 = volumesample(geometry, volumename, pos);
	vector midp1 = set(pos.x+0.5*subteps*v1.x, pos.y+0.5*subteps*v1.y, pos.z+0.5*subteps*v1.z);
	float f2 = volumesample(geometry, volumename, midp1);
	vector midp2 = set(pos.x+0.75*subteps*v2.x, pos.y+0.75*subteps*v2.y, pos.z+0.75*subteps*v2.z);
	float f3 = volumesample(geometry, volumename, midp2);
	float output = c1*f1 + c2*f2 + c3*f3;
	return output;
}
vector GetVectorByRK3(int geometry,string volumename,vector pos,float subteps){
	float c1 = 2.0/9.0*subteps, c2 = 3.0/9.0 * subteps, c3 = 4.0/9.0 * subteps;
	vector input = pos;
	vector v1 = volumesamplev(geometry, volumename, pos);
	vector midp1 = set(pos.x+0.5*subteps*v1.x, pos.y+0.5*subteps*v1.y, pos.z+0.5*subteps*v1.z);
	vector v2 = volumesamplev(geometry, volumename, midp1);
	vector midp2 = set(pos.x+0.75*subteps*v2.x, pos.y+0.75*subteps*v2.y, pos.z+0.75*subteps*v2.z);
	vector v3 = volumesamplev(geometry, volumename, midp2);

	vector output = set(c1*v1.x + c2*v2.x + c3*v3.x,
                        c1*v1.y + c2*v2.y + c3*v3.y,
                        c1*v1.z + c2*v2.z + c3*v3.z);
	return output;
}
void DoubleAdvectVelocity(int geometry,string volumename1, string volumename2,vector pos,float substep){
	vector voxel_index = volumepostoindex(geometry, volumename1,pos);
	vector totalvox = volumeres(geometry, volumename1);

	int evaluations = 8;
	float weight = 1.0/float(evaluations);

	for (int i = 0; i < evaluations; ++i){
		/* code */


	}
	

}

void updateForwardRK3(int geometry,string volumename,vector pos,int isVectorField){
    
    vector index = volumepostoindex(geometry, volumename,pos);
	vector totalvox = volumeres(geometry, volumename);
	
	

	//int numBlocks = ((index.x*index.y*index.z) + 255)/256;






}
void updateBackwardDMC(){}
float estimateVectorDistortion(int geometry,string VDistortion,string Vvolumename1,string Vvolumename2,vector _pos,float substep){
	vector pos = _pos;
	//backward then forward
	vector backward = volumesamplev(geometry, Vvolumename1, pos); 
	vector back_pos = pos + backward * substep;
	vector forward = volumesamplev(geometry, Vvolumename2, back_pos);
	vector fwd_pos = back_pos + forward * substep;
	float dist_bf = distance(pos,fwd_pos);
	//forward then backward
	vector forward = volumesamplev(geometry, Vvolumename2, pos);
	vector fwd_pos = pos + forward * substep;
	vector backward = volumesamplev(geometry, Vvolumename1, fwd_pos); 
	vector back_pos = fwd_pos + backward * substep;
	float dist_fb = distance(pos,back_pos);

	VDistortion = max(dist_bf, dist_fb);
}
float estimateFieldDistortion(int geometry,string FDistortion,string Vvolumename1,string Vvolumename2,vector _pos,float substep){
	vector pos = _pos;
	//backward then forward
	vector backward = volumesamplev(geometry, Vvolumename1, pos); 
	vector back_pos = pos + backward * substep;
	vector forward = volumesamplev(geometry, Vvolumename2, back_pos);
	vector fwd_pos = back_pos + forward * substep;
	float dist_bf = distance(pos,fwd_pos);
	//forward then backward
	vector forward = volumesamplev(geometry, Vvolumename2, pos);
	vector fwd_pos = pos + forward * substep;
	vector backward = volumesamplev(geometry, Vvolumename1, fwd_pos); 
	vector back_pos = fwd_pos + backward * substep;
	float dist_fb = distance(pos,back_pos);

	VDistortion = max(dist_bf, dist_fb);
}
vector accumulateVelocity(int geometry,string volumename,vector pos){
	vector voxel_index = volumepostoindex(geometry, volumename,pos);
	vector vel_acc;
	int evaluations = 8;
	vector sum = set(0, 0, 0);
	float weight = 1.0/float(evaluations);
	 for (int i = -1; i<2; i++){
		 for (int j = -1; i < 2; ++j){
			for (int K = -1; k < 2; ++k){
				if(i!=0&&j!=0&&k!=0){
					vector voxel = set(voxel_index.x + i, voxel_index.y + j, voxel_index.z + k);
					vector vel_around = volumeindex(geometry, volumename, voxel);
					sum = 0.5*sum + 0.5 * vel_around;
					vel_acc += sum;
				}
			}

		 }
		 
	 }
	 return vel_acc;
}
void accumulateField(int geometry,string Fvolumename,vector pos){
	vector voxel_index = volumepostoindex(geometry, Fvolumename,pos);
	vector field_acc;
	int evaluations = 8;
	vector sum = set(0, 0, 0);
	float weight = 1.0/float(evaluations);
	 for (int i = -1; i<2; i++){
		 for (int j = -1; i < 2; ++j){
			for (int K = -1; k < 2; ++k){
				if(i!=0&&j!=0&&k!=0){
					vector voxel = set(voxel_index.x + i, voxel_index.y + j, voxel_index.z + k);
					float field_around = volumeindex(geometry, Fvolumename, voxel);
					sum = 0.5*sum + 0.5 * field_around;
					field_acc += sum;
				}
			}

		 }
		 
	 }
	 return field_acc;
}
void ApplyChangeForwardMap(){}
void advanceBimocq(){
	float proj_coeff = 2.f;
    bool velReinit = false;
    bool scalarReinit = false;

}

void printTestEscaping() {
	string a = "from myLib.h: \\n \\t v\@P, %04.2f";
	printf(a + "\n");
}


#endif // end of include guards