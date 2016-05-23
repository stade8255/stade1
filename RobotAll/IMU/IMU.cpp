#include "stdafx.h"
#include "IMU.h"
#include "math.h"



IMU ::IMU (void){
 
	absangle  = 1 ;   //使用絕對角度
	
	calicount  = 1000 ;   //cali的筆數
	
	AngularVel_bound = 0 ;
	//adamssimcount = 0;

	Ts =0.005;             //IMU sampling freqency 目前為 200 HZ

	microstraincalix = 0;//0.00847 ; //cali值  與 microstrain MIP 比較    直接平移一個量

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
	
		anglex[i]=0;    // vel 積分
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
	
		finalanglex[i]=0;  //complementary 算出  
		finalangley[i]=0;   
		finalanglez[i]=0;
	
		finalanglex_test[i]=0;

		AngularVelx[i]=0;	
		AngularVely[i]=0;	
		AngularVelz[i]=0;	
	
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
	
void  IMU :: integation(float *AngularVelx ,float *anglex , float *AngularVely ,float *angley ,float *AngularVelz ,float *anglez ){
                   
				   
				   AngularVelbiasx1  = (*AngularVelx)   -AngularVelbiasx;  
			       AngularVelbiasx2  = *(AngularVelx-1) -AngularVelbiasx;
			   
				   AngularVelbiasy1  = (*AngularVely) -AngularVelbiasy;  
			       AngularVelbiasy2  = *(AngularVely-1) -AngularVelbiasy;

				   AngularVelbiasz1  = (*AngularVelz) -AngularVelbiasz;  
			       AngularVelbiasz2  = *(AngularVelz-1) -AngularVelbiasz;   // AngularVel  bias 

				    
		      *anglex=  *(anglex-1)  +  0.5*Ts* (  AngularVelbiasx1) +   0.5*Ts* (  AngularVelbiasx2 )    ; 
              *angley=  *(angley-1)  +  0.5*Ts* (  AngularVelbiasy1) +   0.5*Ts* (  AngularVelbiasy2 )    ; 
			  *anglez=  *(anglez-1)  +  0.5*Ts* (  AngularVelbiasz1) +   0.5*Ts* (  AngularVelbiasz2 )  ; 
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

	

	* ang =  *ang - bias ;  //扣掉相對角度的bias



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
	
	i3dmgx3_openPort(portNum, 115200, 8, 0, 1, 1024, 1024);//開port
	
	
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
			error_record++;

		if (cmd_return == MODE)
		{
			//move to the acceleration position and print the data		 
			IMU_Lock=true;

			accelx[count] =   Record.setA[1]*9.780318*1000 ;//   單位mm/s^2
			accely[count] =   Record.setA[0]*9.780318*1000 ;
			accelz[count] =   Record.setA[2]*9.780318*1000 ;	

			AngularVelx[count] =  Record.setB[0];//     單位rad/s
			AngularVely[count] =  Record.setB[1] ;
			AngularVelz[count] =  Record.setB[2] ;	
	             
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
			
			AngularVel_bound = 2.5;


		    //絕對角度  由rotation matrix算出
			//absanglex[count]   = - asin(   accelx[count] /0.99) -  abscalix;
			//absangley[count]   =   asin(   accely[count] /0.99) -  abscaliy;  
	  
			absanglex[count] = atan2(M23[count], M33[count]);
			absangley[count] = asin(-M13[count]);
			absanglez[count] = atan2(M12[count], M11[count]);
				  
			//避免角度爆掉  設 absangle upper bound = 1.55  and velocity upper bound= 2.5 
				  
			if(abs(absanglex[count])>=(PI/2))				 
				absanglex[count] = (accelx[count]/abs(accelx[count]))*(PI/2);
				 		  
			if(abs(absangley[count])>=(PI/2))  
				absangley[count] = (accely[count]/abs(accely[count]))*(PI/2);
			 						
			if(abs(AngularVelx[count])>AngularVel_bound)
				AngularVelx[count] = (AngularVelx[count]/abs(AngularVelx[count]))*AngularVel_bound;
				  		  
			if(abs(AngularVely[count])>AngularVel_bound)
				AngularVely[count] = (AngularVely[count]/abs(AngularVely[count]))*AngularVel_bound;

			//相對角度的bias累積 在cali前
			anglebiasx = anglebiasx + anglex[count-1];
			anglebiasy = anglebiasy + angley[count-1];
			anglebiasz = anglebiasz + anglez[count-1];

				  
			//絕對角度的bias累積  在cali前
			absxbias = absxbias + absanglex[count]; 
			absybias = absybias + absangley[count]; 


		    //當count = calicount時  將累積bias填入buffer內 						 
		    if (count == 0.5*calicount)
		    {    
			    if (absangle == 1)
			    { 
				   //使用絕對角度 ==> 不cali
					abscalix = 0; 
					abscaliy = 0;
		        }
	    		else
				{  
					abscalix =  absxbias /(0.5*calicount);   //使用相對角度 ==> cali
					abscaliy =  absybias /(0.5*calicount);
		        }
							  
				for (int i = 0 ; i<(0.5*calicount);i++)
				{  
					//將AngularVel累積誤差bias消去
					AngularVelbiasx  = AngularVelbiasx + AngularVelx[i] ;
					AngularVelbiasy  = AngularVelbiasy + AngularVely[i] ;
					AngularVelbiasz  = AngularVelbiasz + AngularVelz[i] ;
				}		
				AngularVelbiasx = AngularVelbiasx /(calicount*0.5);
				AngularVelbiasy = AngularVelbiasy /(calicount*0.5);
				AngularVelbiasz = AngularVelbiasz /(calicount*0.5);
		    }	

		    if ( count == calicount )//先讓AngularVel bias 消除   往後0.5cali count 再把角度bias 減掉
		    {
		  	    anglecalix  =  anglebiasx /(0.5*calicount);
			    anglecaliy   = anglebiasy /(0.5*calicount);
			    anglecaliz   = anglebiasz /(0.5*calicount); 
		    }

			if (count > 0.5*calicount)
			{
				integation (AngularVelx +count,anglex +count,AngularVely +count,angley +count,AngularVelz +count,anglez+count);
			}	

			//finalanglex (相對角度)   absanglex (絕對角度) 

		    complementaryfilter(  anglex +count  ,  absanglex+count  ,filterx +count, filterabsx +count ,  anglefilterx+count , anglecalix);
		    complementaryfilter(  angley +count  ,  absangley+count  ,filtery +count, filterabsy +count ,  anglefiltery+count , anglecaliy);        


		    finalanglex[count] = anglefilterx[count] + microstraincalix ;  //加入 cali值  與 microstrain MIP 比較
		    finalangley[count] = anglefiltery[count] + microstraincaliy ;

			//print 角度 (度度制) cali完後就不顯示 比較不占資源
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
		  
		   


		          //adams讀入  accel (mm/s)     angvel(rad/s)
            /*     accelx[count] = accelx[count]*0.001;
				 accely[count] = accely[count]*0.001;*/
						



		          //絕對角度  由accel分量算出
				  absanglex[adamssimcount]   =   asin(   0.001*accelx[adamssimcount] /9.9) -  abscalix;
				  absangley[adamssimcount]   =   asin(   0.001*accely[adamssimcount] /9.9) -  abscaliy;  
				  
					
				  //相對角度的bias累積 在cali前
				  anglebiasx    =  anglebiasx  +   anglex[adamssimcount-1] ;
				  anglebiasy    =  anglebiasy  +   angley[adamssimcount-1] ;
				  anglebiasz    =  anglebiasz  +   angley[adamssimcount-1] ;
              
				  
				  //絕對角度的bias累積  在cali前
				  absxbias =  absxbias  +  absanglex[adamssimcount-1] ; 
			      absybias =  absybias  +  absangley [adamssimcount-1]; 





   //當count = calicount時  將累積bias填入buffer內 						 
   if (adamssimcount ==0.5*calicount){    
	                          
	     
		  if (absangle == 1){ //使用絕對角度 ==> 不cali

	    abscalix = 0; 
	    abscaliy = 0;
		                    }
	    
		  else {  
		abscalix =  absxbias /(0.5*calicount);   //使用相對角度 ==> cali
	    abscaliy =  absybias /(0.5*calicount);

		       }





							  
		for (int i = 0 ; i<(0.5*calicount);i++){  //將AngularVel累積誤差bias消去
		AngularVelbiasx  = AngularVelbiasx + AngularVelx[i] ;
		AngularVelbiasy  = AngularVelbiasy + AngularVely[i] ;
		AngularVelbiasz  = AngularVelbiasz + AngularVelz[i] ;
																				
		}
							
	    AngularVelbiasx = AngularVelbiasx /(calicount*0.5);
	    AngularVelbiasy = AngularVelbiasy /(calicount*0.5);
	    AngularVelbiasz = AngularVelbiasz /(calicount*0.5);
        }	



 

   if (adamssimcount == calicount )//先讓AngularVel bias 消除   往後0.5cali count 再把角度bias 減掉

   
   
   {
   anglecalix  =  anglebiasx /(0.5*calicount);
   anglecaliy   = anglebiasy /(0.5*calicount);
   anglecaliz   = anglebiasz /(0.5*calicount); 
   }




   if (adamssimcount > 0.5*calicount){

	   integation (AngularVelx +adamssimcount,anglex +adamssimcount,AngularVely +adamssimcount,angley +adamssimcount,AngularVelz +adamssimcount,anglez+adamssimcount);

	   
	   } ;					 



   //finalanglex (相對角度)   absanglex (絕對角度) 


   complementaryfilter(  anglex +adamssimcount  ,  absanglex+adamssimcount  ,filterx +adamssimcount, filterabsx +adamssimcount ,  anglefilterx+adamssimcount , anglecalix);
   complementaryfilter(  angley +adamssimcount  ,  absangley+adamssimcount  ,filtery +adamssimcount, filterabsy +adamssimcount ,  anglefiltery+adamssimcount , anglecaliy);        




   finalanglex[adamssimcount]  = anglefilterx[adamssimcount] + microstraincalix ;  //加入 cali值  與 microstrain MIP 比較

   finalangley[adamssimcount]  = anglefiltery[adamssimcount] + microstraincaliy ;      

   
   
   //0418
   // IMU x sagittal direction(roll)    y lateral direction(pitch) 
   // 需乘上 rotation matrix 將 IMU accel 在 local frame 轉到 world frame 
   
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

