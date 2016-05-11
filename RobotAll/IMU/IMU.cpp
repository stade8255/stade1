#include "stdafx.h"
#include "IMU.h"
#include "math.h"



IMU ::IMU (void){
 
	absangle  = 1 ;   //�ϥε��﨤��
	
	calicount  = 1000 ;   //cali������
	
	vel_bound = 0 ;
	//adamssimcount = 0;

	Ts =0.005;             //IMU sampling freqency �ثe�� 200 HZ

	microstraincalix = 0;//0.00847 ; //cali��  �P microstrain MIP ���    ���������@�Ӷq

	microstraincaliy = 0;//0.0171  ; 
	
	IMU_plot = true;
		
	IMU_Lock=false;

	anglexbias = 0 ;
	angleybias = 0 ; 
	absxbias  = 0 ;
	absybias  = 0 ;


	abscalix = 0 ;
	abscaliy = 0 ;
	anglecalix = 0 ;  
	anglecaliy = 0 ;  
	anglecaliz = 0 ;  

	
	for(int i = 0; i<10000 ; i++)
	{
	
		anglex[i]=0;    // vel �n��
		angley[i]=0;
		anglez[i]=0;
	
		anglefilterx[i]=0;
		anglefiltery[i]=0;
		anglefilterz[i]=0;
	
		filterx[i]=0;
		filtery[i]=0;
		filterz[i]=0;
	
		filterabsx[i]=0;
		filterabsy[i]=0;
		
		absanglex[i]=0;
		absangley[i]=0;
		absanglez[i]=0;
	
		finalanglex[i]=0;  //complementary ��X  
		finalangley[i]=0;   
		finalanglez[i]=0;
	
		finalanglex_test[i]=0;

		velx[i]=0;	
		vely[i]=0;	
		velz[i]=0;	
	
		accelx[i]=0;
		accely[i]=0;
		accelz[i]=0;

		magx[i]=0;
		magy[i]=0;
		magz[i]=0;

		M11[i]=0;
		M12[i]=0;
		M13[i]=0;
		M21[i]=0;
		M22[i]=0;
		M23[i]=0;
		M31[i]=0;
		M32[i]=0;
		M33[i]=0;
	}
	
}
	
void  IMU :: integation(float *velx ,float *anglex , float *vely ,float *angley ,float *velz ,float *anglez ){
                   
				   
				   velbiasx1  = (*velx)   -velbiasx;  
			       velbiasx2  = *(velx-1) -velbiasx;
			   
				   velbiasy1  = (*vely) -velbiasy;  
			       velbiasy2  = *(vely-1) -velbiasy;

				   velbiasz1  = (*velz) -velbiasz;  
			       velbiasz2  = *(velz-1) -velbiasz;   // vel  bias 

				    
		      *anglex=  *(anglex-1)  +  0.5*Ts* (  velbiasx1) +   0.5*Ts* (  velbiasx2 )    ; 
              *angley=  *(angley-1)  +  0.5*Ts* (  velbiasy1) +   0.5*Ts* (  velbiasy2 )    ; 
			  *anglez=  *(anglez-1)  +  0.5*Ts* (  velbiasz1) +   0.5*Ts* (  velbiasz2 )  ; 
  }


void SetDataRate(int portNum)
{
	int status = 0, Value = 0, Ival =0;
	int chOption = 0;
	int rDataRate = 0;
	int mDataRate = 0;
	int sDataRate = 0;
	int valCheck = 0;
	BYTE Record[20];
	char AddA = 0xFC;
	char AddB = 0xA2;
    unsigned short nvalS  = 0;
	char Jjunk[20];
	purge_port(portNum);
	Value == 0 ;
	
	
	WriteDataRate(portNum, 200, &Record[0]);

};

void IMU :: complementaryfilter(float   * ang ,  float * abs , float * filterang , float *filterabs , float *filter , float bias  ){

	

	* ang =  *ang - bias ;  //�����۹﨤�ת�bias



	* filterang=  0.81425 * * ang -3.257 *  * (ang-1) + 4.8855 * * (ang-2) -3.257 *  * (ang-3) + 0.81425 ** (ang-4)
		- ( -3.5897* * (filterang-1) + 4.8513 * * (filterang-2) -2.9241** (filterang-3) +0.66301 ** (filterang-4)) ;



	* filterabs =  3.1239e-005 * * abs + 0.00012496 *  * (abs-1) + 0.00018743 * * (abs-2) +0.00012496 *  * (abs-3) + 3.1239e-005 ** (abs-4)
		- ( -3.5897 * * (filterabs-1) + 4.8513 * * (filterabs-2) -2.9241 ** (filterabs-3) +0.66301 ** (filterabs-4)) ; 



	*filter   =  * filterang + * filterabs ; 


}



void IMU :: ReadContinuousData(int portNum)
{
	
	BOOL bStopContinuous = FALSE;
	DWORD dwcharsRead = 0;
	I3dmgx3Set Record;
	BYTE cmd_return = {0};
	int Curs_posY = 0;
	int Curs_posX = 0;
	int status    = 0;
	int error_record = 0;
	char consoleBuff[60] = {0};
	long valid_rec = 0;
	unsigned char error_cmd;
	double MODE = 0xCC;
	
	i3dmgx3_openPort(portNum, 115200, 8, 0, 1, 1024, 1024);//�}port
	
	
	//put the node in continuous mode
	status = SetContinuousMode(portNum, MODE);
	
	
	
	if (IMU_plot==true)
	{
	printf("_________________________________  angle_________________________________\n");
	printf("          ROLL                   PITCH                         COUNT         \n");
	printf("\n");
	
	getConXY(&Curs_posX, &Curs_posY); 
	printf("\n\n\n\n\n");
	}
	

	//continue until the user hits the s key
	while(!bStopContinuous)
	{
		if(ReadNextRecord(portNum, &Record, &cmd_return) != SUCCESS)
		{
			error_record++;
			cout<<"falseqq"<<error_record<<endl;
		}

		if (cmd_return == MODE)
		{
			//move to the acceleration position and print the data		 
			IMU_Lock=true;

			accelx[count] =   Record.setA[1]*9.780318 ;//   ���m/s^2
			accely[count] =   Record.setA[0]*9.780318 ;
			accelz[count] =   Record.setA[2]*9.780318 ;	

			velx[count] =  Record.setB[0];//     ���rad/s
			vely[count] =  Record.setB[1] ;
			velz[count] =  Record.setB[2] ;	
	             
		/*	magx[count] = Record.setC[1];
			magy[count] = Record.setC[0];
			magz[count] = -Record.setC[2];*/
					  
			M11[count] = Record.setD[0];
			M12[count] = Record.setD[1];
			M13[count] = Record.setD[2];

			M21[count] = Record.setE[0];
			M22[count] = Record.setE[1];
			M23[count] = Record.setE[2];

			M31[count] = Record.setF[0];
			M32[count] = Record.setF[1];
			M33[count] = Record.setF[2];
			
			vel_bound = 0.3;


		    //���﨤��  ��rotation matrix��X
			//absanglex[count]   = - asin(   accelx[count] /0.99) -  abscalix;
			//absangley[count]   =   asin(   accely[count] /0.99) -  abscaliy;  
	  
			absanglex[count] = atan2(M23[count], M33[count])  -  abscalix;
			absangley[count] = asin(-M13[count]) -  abscaliy;
			absanglez[count] = atan2(M12[count], M11[count]);
				  
				  //�קK�����z��  �] absangle upper bound = 1.55  and velocity upper bound= 2.5 
				  
				  if(abs(absanglex[count])>=(PI/2))				 
					  absanglex[count] = (accelx[count]/abs(accelx[count]))*(PI/2);
				 		  
				  if(abs(absangley[count])>=(PI/2))
				  
					  absangley[count] = (accely[count]/abs(accely[count]))*(PI/2);
			 						
				  if(abs(velx[count])>vel_bound)
					   velx[count] = (velx[count]/abs(velx[count]))*vel_bound;
				  		  
				  if(abs(vely[count])>vel_bound)
					   vely[count] = (vely[count]/abs(vely[count]))*vel_bound;




				  //�۹﨤�ת�bias�ֿn �bcali�e
				  anglebiasx    =  anglebiasx  +   anglex[count-1] ;
				  anglebiasy    =  anglebiasy  +   angley[count-1] ;
				  anglebiasz    =  anglebiasz  +   angley[count-1] ;

				  
				  //���﨤�ת�bias�ֿn  �bcali�e
				  absxbias =  absxbias  +  absanglex[count] ; 
			      absybias =  absybias  +  absangley [count]; 





   //��count = calicount��  �N�ֿnbias��Jbuffer�� 						 
   if (count ==0.5*calicount){    
	                          
	     
		  if (absangle == 1){ //�ϥε��﨤�� ==> ��cali

	    abscalix = 0; 
	    abscaliy = 0;
		                    }
	    
		  else {  
		abscalix =  absxbias /(0.5*calicount);   //�ϥά۹﨤�� ==> cali
	    abscaliy =  absybias /(0.5*calicount);

		       }





							  
		for (int i = 0 ; i<(0.5*calicount);i++){  //�Nvel�ֿn�~�tbias���h
		velbiasx  = velbiasx + velx[i] ;
		velbiasy  = velbiasy + vely[i] ;
		velbiasz  = velbiasz + velz[i] ;
																				
		}
							
	    velbiasx = velbiasx /(calicount*0.5);
	    velbiasy = velbiasy /(calicount*0.5);
	    velbiasz = velbiasz /(calicount*0.5);
        }	



 

   if (count == calicount )//����vel bias ����   ����0.5cali count �A�⨤��bias �

   
   
   {
   anglecalix  =  anglebiasx /(0.5*calicount);
   anglecaliy   = anglebiasy /(0.5*calicount);
   anglecaliz   = anglebiasz /(0.5*calicount); 
   }




   if (count > 0.5*calicount){

	   integation (velx +count,anglex +count,vely +count,angley +count,velz +count,anglez+count);

	   
	   }	
   
   
   ;					 



   //finalanglex (�۹﨤��)   absanglex (���﨤��) 


   complementaryfilter(  anglex +count  ,  absanglex+count  ,filterx +count, filterabsx +count ,  anglefilterx+count , anglecalix);
   complementaryfilter(  angley +count  ,  absangley+count  ,filtery +count, filterabsy +count ,  anglefiltery+count , anglecaliy);        


   finalanglex[count]  = anglefilterx[count] + microstraincalix ;  //�[�J cali��  �P microstrain MIP ���

   finalangley[count]  = anglefiltery[count] + microstraincaliy ;



   //print ���� (�׫ר�) cali����N����� ������e�귽
   

   
   if (IMU_plot==true)
   {
	   sprintf(consoleBuff, "\t%f\t\t%f\t\t\t%d\n", finalanglex[count]*180/3.14,finalangley[count]*180/3.14 , count );
	   setConXY(Curs_posX, Curs_posY ,  &consoleBuff[0]);
   }




   valid_rec++;
   count++; 
   
   		         IMU_Lock=false;

			}

	}

}


void IMU ::adamssim(int gFlagSimulation,int adamssimcount){
	if (gFlagSimulation ==1 ) //adams sim
	{     
		///motion control thread begins in 1000counts
		  
		   


		          //adamsŪ�J  accel (mm/s)     angvel(rad/s)
            /*     accelx[count] = accelx[count]*0.001;
				 accely[count] = accely[count]*0.001;*/
						



		          //���﨤��  ��accel���q��X
				  absanglex[adamssimcount]   =   asin(   0.001*accelx[adamssimcount] /9.9) -  abscalix;
				  absangley[adamssimcount]   =   asin(   0.001*accely[adamssimcount] /9.9) -  abscaliy;  
				  
					
				  //�۹﨤�ת�bias�ֿn �bcali�e
				  anglebiasx    =  anglebiasx  +   anglex[adamssimcount-1] ;
				  anglebiasy    =  anglebiasy  +   angley[adamssimcount-1] ;
				  anglebiasz    =  anglebiasz  +   angley[adamssimcount-1] ;
              
				  
				  //���﨤�ת�bias�ֿn  �bcali�e
				  absxbias =  absxbias  +  absanglex[adamssimcount-1] ; 
			      absybias =  absybias  +  absangley [adamssimcount-1]; 





   //��count = calicount��  �N�ֿnbias��Jbuffer�� 						 
   if (adamssimcount ==0.5*calicount){    
	                          
	     
		  if (absangle == 1){ //�ϥε��﨤�� ==> ��cali

	    abscalix = 0; 
	    abscaliy = 0;
		                    }
	    
		  else {  
		abscalix =  absxbias /(0.5*calicount);   //�ϥά۹﨤�� ==> cali
	    abscaliy =  absybias /(0.5*calicount);

		       }





							  
		for (int i = 0 ; i<(0.5*calicount);i++){  //�Nvel�ֿn�~�tbias���h
		velbiasx  = velbiasx + velx[i] ;
		velbiasy  = velbiasy + vely[i] ;
		velbiasz  = velbiasz + velz[i] ;
																				
		}
							
	    velbiasx = velbiasx /(calicount*0.5);
	    velbiasy = velbiasy /(calicount*0.5);
	    velbiasz = velbiasz /(calicount*0.5);
        }	



 

   if (adamssimcount == calicount )//����vel bias ����   ����0.5cali count �A�⨤��bias �

   
   
   {
   anglecalix  =  anglebiasx /(0.5*calicount);
   anglecaliy   = anglebiasy /(0.5*calicount);
   anglecaliz   = anglebiasz /(0.5*calicount); 
   }




   if (adamssimcount > 0.5*calicount){

	   integation (velx +adamssimcount,anglex +adamssimcount,vely +adamssimcount,angley +adamssimcount,velz +adamssimcount,anglez+adamssimcount);

	   
	   } ;					 



   //finalanglex (�۹﨤��)   absanglex (���﨤��) 


   complementaryfilter(  anglex +adamssimcount  ,  absanglex+adamssimcount  ,filterx +adamssimcount, filterabsx +adamssimcount ,  anglefilterx+adamssimcount , anglecalix);
   complementaryfilter(  angley +adamssimcount  ,  absangley+adamssimcount  ,filtery +adamssimcount, filterabsy +adamssimcount ,  anglefiltery+adamssimcount , anglecaliy);        




   finalanglex[adamssimcount]  = anglefilterx[adamssimcount] + microstraincalix ;  //�[�J cali��  �P microstrain MIP ���

   finalangley[adamssimcount]  = anglefiltery[adamssimcount] + microstraincaliy ;      

   
   
   //0418
   // IMU x sagittal direction(roll)    y lateral direction(pitch) 
   // �ݭ��W rotation matrix �N IMU accel �b local frame ��� world frame 
   
   //ax = ax*cos(pitch) + ay*sin(pitch) sin(roll)  + sin(pitch) sin(roll)
   //ay = cos(roll)*ay - sin(roll)
   //az = 1

   //IMU_roll  =   finalanglex[count] ;
   //IMU_pitch =   finalangley[count] ;



   //accelx[count] =   accelx[count] * cos( IMU_pitch) + accely[count]*sin( IMU_pitch)*sin(IMU_roll)+sin(IMU_pitch)*sin(IMU_roll)*1;
   //accely[count] =   accely[count] * cos(IMU_roll) -sin(IMU_roll)*1;
    
   //cout<<IMU_roll<<" " << IMU_pitch << endl;

  
   
   //0418



    adamssimcount++; 
   
 }

}








IMU ::~IMU (void){} ;

