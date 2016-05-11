#include "StdAfx.h"
#include "SoundPlay.h"
#include "iostream"
#include "stdlib.h"
using namespace std;


CSoundPlay::CSoundPlay(void)
: system(NULL)
, channel(NULL)
{
	Initialize();
}

CSoundPlay::~CSoundPlay(void)
{
	//ReleaseSoundFile();
}

bool CSoundPlay::Initialize(void)
{
	FMOD_RESULT result;
	unsigned int      version;

	/*
        Create a System object and initialize.
    */
    result = FMOD::System_Create(&system);
    ERRCHECK(result);

    result = system->getVersion(&version);
    ERRCHECK(result);

   /* if (version < FMOD_VERSION)
    {
        printf("Error!  You are using an old version of FMOD %08x.  This program requires %08x\n", version, FMOD_VERSION);
        return 0;
    }*/

    result = system->init(32, FMOD_INIT_NORMAL, 0);
    ERRCHECK(result);

	// channel
	channel = 0;

	// load sound file

	//sprintf(FileName,"./Output4.wav");

	//LoadSoundFile(FileName);
	//LoadSoundFile("./Output4.wav",speaker.Sound1);

	return true;
}

bool CSoundPlay::LoadSoundFile(const char* FileName)
{
	// loading the sound files

	FMOD_RESULT       result;

	result = system->createSound(FileName, FMOD_HARDWARE, 0, &Sound1);
	//result = system->createSound(FileName, FMOD_HARDWARE, 0, &Sound1);

    ERRCHECK(result);

	return true;
}



bool CSoundPlay::PlaySoundByFile(FMOD::Sound *S1)
{
	FMOD_RESULT result;
	result = system->playSound(FMOD_CHANNEL_FREE, S1, false, &channel);
    ERRCHECK(result);
	return false;
}


bool CSoundPlay::ReleaseSoundFile(FMOD::Sound *S1)
{
	FMOD_RESULT result;
	
		result = S1->release();
		 ERRCHECK(result);


	return true;
}

//20111018�u��|���{��//

int CSoundPlay::TTSEngineTest(wchar_t *pszInputString, wchar_t *szVoiceEngine, int par_speed, int iMode, wchar_t *szOutputFileName, SPSTREAMFORMAT iOutputFileFormat)
{
	HRESULT hr;

	CComPtr<IEnumSpObjectTokens>        cpEnum;
	//�]�w VoiceEngine
	CComPtr <ISpObjectToken> cpVoiceToken;
	CComPtr <ISpObjectToken> cpAudioToken;

	SpEnumTokens( SPCAT_AUDIOOUT, NULL, NULL, &cpEnum );


	hr = SpFindBestToken(SPCAT_VOICES, szVoiceEngine, NULL, &cpVoiceToken);									
	if (!SUCCEEDED(hr)) //�]�w VoiceEngine ����
	{
		printf("Error(SpFindBestToken): VoiceEngine Setting Failed...\n");
		return FALSE;
	}
	cpVoice->SetVoice(cpVoiceToken);

	//�]�w WaveFormat
	CSpStreamFormat cAudioFormat;
	hr = cAudioFormat.AssignFormat(iOutputFileFormat);
	if(!SUCCEEDED(hr))
	{
		printf("Error(AssignFormat): AssignFormat Failed...\n");
		return FALSE;
	}
	
	//����n����X�覡
	if (iMode==0) //iMode==0 �N�n����X�ܳ�z
	{
		//set the output to the audio device
		if( SUCCEEDED( hr ) )
		{
			cpEnum->Item(0, &cpAudioToken);
			cpVoice->SetOutput(cpAudioToken, TRUE);
			//cpVoice->Speak(pszInputString, SPF_ASYNC | SPF_IS_XML, NULL);//�qSPF_ASYNC�令SPF_DEFAULT�N�i�H�ΤF�A��]����

			long speed;
			cpVoice->GetRate(&speed);
			speed += par_speed;
			cpVoice->SetRate(speed);

			cpVoice->Speak(pszInputString, SPF_DEFAULT | SPF_IS_XML, NULL);
		}
		else
		{
			printf("ERROR(SetFormat)\n"); 
			return FALSE;
		}
	}
	else //iMode!=0 �N�n����X��Wave�ɮ�, �ɮ׮榡��iOutputFileFormat
	{	
		CComPtr	<ISpStream>	cpWaveStream;
		
		hr = SPBindToFile(szOutputFileName, SPFM_CREATE_ALWAYS, &cpWaveStream, &cAudioFormat.FormatId(), cAudioFormat.WaveFormatExPtr() );
		
		if( SUCCEEDED( hr ) )  //���pSPBindToFile is OK.
		{
			//set the output to cpWaveStream so that the output audio data will be stored in cpWaveStream
			cpVoice->SetOutput(cpWaveStream, TRUE);	
	
			long speed;
			cpVoice->GetRate(&speed);
			speed += par_speed;
			cpVoice->SetRate(speed);

			cpVoice->Speak(pszInputString, SPF_DEFAULT | SPF_IS_XML, NULL);				
			if ( cpWaveStream )
			{
				cpWaveStream->Close();
				cpWaveStream.Release();
			}													
		}		
		else
		{
			printf("ERROR(SPBindToFile)\n"); 
			return FALSE;
		}// end of if( SUCCEEDED( hr ) )  //���pSPBindToFile is OK.		
	}//end of if (iMode==0) //iMode==0 �N�n����X�ܳ�z	

	//system("Pause");
	//cpVoiceToken.Release();
	//system("pause");
	return TRUE;
}


bool CSoundPlay::speak(char* YourInputText)
{
	/*���F�ഫ char* �� System ���ϥΪ� wchar (for converting the C-Style char to System Char)***/

	CString YourInputText_CString;

	YourInputText_CString.Format("%s",YourInputText);

	/*��r�ഫ�@�y�����{�� Text-To-Speech procedures*/

	//ISpVoice * pVoice = NULL;  /*1. �ǳƤ@�� NULL�F�� pointercreate a pointer*/

	if (FAILED(CoInitialize(NULL))) /* 2. ���ժ�l�� COM Check if we can Initialize the COM*/

	{

	printf("Error to intiliaze COM"); return false;

	}

	 /*3. ��l�� Text-To-Speech object (Initialize Text-To-Speech object)*/

	//HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
	HRESULT hr;
		// Initialize SAPI 
	printf("TTS SAPI Initializing ...... ");
	if (FAILED(::CoInitialize(NULL)))
	{ 
		printf("ERROR to initialize SAPI\n"); 
		return FALSE;
	}		
	printf("Initialized OK!\n\n");
	//Create Instance for cpVoice
	hr = cpVoice.CoCreateInstance( CLSID_SpVoice );	
	if (!SUCCEEDED(hr)) //���pCoCreateInstance is not OK.
	{ 
		printf("ERROR(CoCreateInstance)\n"); 
		return FALSE;
	}

	if( SUCCEEDED( hr ) ) /*4. �Y��l�� TTS engine ���\ (If initialization of TTS succeeded)*/

	{

		/*5. Ū�X "Your Input is" (speaks the words "Your Input is") */

		//hr = pVoice->Speak(L"Your Input is", 0, NULL);

		/* 5. Ū�X YourInputText_CString �����e (speaks the content of ) YourInputText_CString */
		
		//debug use
		 //cout<<YourInputText_CString<<"\n\n";
		//hr = pVoice->Speak(YourInputText_CString.AllocSysString(),SPF_ASYNC | SPF_IS_XML,NULL);
			//�]�wpTTS�X���Ѽ�
			//wchar_t szVoiceEngine[] = L"Name=ITRI Theresa pTTS (PMC)"; // ITRI Female Voice Engine
			wchar_t szVoiceEngine[] = L"Name=ITRI Bruce pTTS (PMC)"; // ITRI Male Voice Engine: "Name=ITRI Bruce pTTS (PMC)"
																	 // ITRI Female Voice Engine: "Name=ITRI Theresa pTTS (PMC)"
			//pTTS �Ǧ� - �X�z�d��:-10��10��������ƭ�, �ȶV�j�h�����V���F�Ϥ��h�����V�C
			int iLevel = 3;
			//pTTS ��V - �X�z�d��:0��2��������ƭ� (0�G���`, 1�G�������H, 2�G���~��H������)
			int iDirection = 0;
			//pTTS ��V - �X�z�d��:0��20��������ƭ�, �ȶV�j�h���y���V����F�Ϥ��h�V�ͥ���
			int iVariance = 8;
			//�N������X�ܳ�z(iMode = 0)�ο�X�n���ɮ�(iMode = 1)
			int iMode = 0;	
			//��XWave�ɮ׮榡: �i�d�� Microsoft Speech SDK ��󻡩� (help SPSTREAMFORMAT)
			SPSTREAMFORMAT iOutputFileFormat = SPSF_16kHz16BitMono;
			//SPSTREAMFORMAT iOutputFileFormat = SPSF_8kHz16BitMono;
			//��XWave�ɮצW��
			//wchar_t szOutputFileName[] = L"�ۧڤ���.wav";	
			//wchar_t szOutputFileName[] = L"�̶}�l���w���.wav";
			//wchar_t szOutputFileName[] = L"���i����ɭԪ��ۧڤ���.wav";
			wchar_t szOutputFileName[] = L"�����H�X��.wav";
		wchar_t *pszInputString = YourInputText_CString.AllocSysString();	// ��J����r��
		wchar_t *pszTagOutputString;				// ��X�����J�аO�y������r��
		int		iTagAdditionLen = 64;				// ���J�аO�y���ҷs�W���r�����

		pszTagOutputString = new wchar_t [wcslen(pszInputString)+iTagAdditionLen];

		swprintf(pszTagOutputString, L"<PitchModify para=\"%d;%d;%d\"/>%s", iLevel, iDirection, iVariance, pszInputString);

		TTSEngineTest(pszTagOutputString, szVoiceEngine,3, iMode, szOutputFileName, iOutputFileFormat);

		delete [] pszTagOutputString;
		

		 // pVoice->Release(); //6. ����TTS Engine (release the Text2Speech Engine after using)

		//pVoice = NULL;//7. �b����e set NULL(Set to NULL before Uninitialize the COM)

	}

	CoUninitialize(); // 8. �Ѱ� COM (Uninitialize the COM)

	return true;
}
bool CSoundPlay::speak(char* YourInputText,int iSpeed)
{
	/*���F�ഫ char* �� System ���ϥΪ� wchar (for converting the C-Style char to System Char)***/

	CString YourInputText_CString;

	YourInputText_CString.Format("%s",YourInputText);

	/*��r�ഫ�@�y�����{�� Text-To-Speech procedures*/

	//ISpVoice * pVoice = NULL;  /*1. �ǳƤ@�� NULL�F�� pointercreate a pointer*/

	if (FAILED(CoInitialize(NULL))) /* 2. ���ժ�l�� COM Check if we can Initialize the COM*/

	{

	printf("Error to intiliaze COM"); return false;

	}

	 /*3. ��l�� Text-To-Speech object (Initialize Text-To-Speech object)*/

	//HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
	HRESULT hr;
		// Initialize SAPI 
	printf("TTS SAPI Initializing ...... ");
	if (FAILED(::CoInitialize(NULL)))
	{ 
		printf("ERROR to initialize SAPI\n"); 
		return FALSE;
	}		
	printf("Initialized OK!\n\n");
	//Create Instance for cpVoice
	hr = cpVoice.CoCreateInstance( CLSID_SpVoice );	
	if (!SUCCEEDED(hr)) //���pCoCreateInstance is not OK.
	{ 
		printf("ERROR(CoCreateInstance)\n"); 
		return FALSE;
	}

	if( SUCCEEDED( hr ) ) /*4. �Y��l�� TTS engine ���\ (If initialization of TTS succeeded)*/

	{

		/*5. Ū�X "Your Input is" (speaks the words "Your Input is") */

		//hr = pVoice->Speak(L"Your Input is", 0, NULL);

		/* 5. Ū�X YourInputText_CString �����e (speaks the content of ) YourInputText_CString */
		
		//debug use
		 //cout<<YourInputText_CString<<"\n\n";
		//hr = pVoice->Speak(YourInputText_CString.AllocSysString(),SPF_ASYNC | SPF_IS_XML,NULL);
			//�]�wpTTS�X���Ѽ�
			//wchar_t szVoiceEngine[] = L"Name=ITRI Theresa pTTS (PMC)"; // ITRI Female Voice Engine
			wchar_t szVoiceEngine[] = L"Name=ITRI Bruce pTTS (PMC)"; // ITRI Male Voice Engine: "Name=ITRI Bruce pTTS (PMC)"
																	 // ITRI Female Voice Engine: "Name=ITRI Theresa pTTS (PMC)"
			//pTTS �Ǧ� - �X�z�d��:-10��10��������ƭ�, �ȶV�j�h�����V���F�Ϥ��h�����V�C
			int iLevel = 3;
			//pTTS ��V - �X�z�d��:0��2��������ƭ� (0�G���`, 1�G�������H, 2�G���~��H������)
			int iDirection = 0;
			//pTTS ��V - �X�z�d��:0��20��������ƭ�, �ȶV�j�h���y���V����F�Ϥ��h�V�ͥ���
			int iVariance = 8;
			//�N������X�ܳ�z(iMode = 0)�ο�X�n���ɮ�(iMode = 1)
			int iMode = 1;	
			//��XWave�ɮ׮榡: �i�d�� Microsoft Speech SDK ��󻡩� (help SPSTREAMFORMAT)
			SPSTREAMFORMAT iOutputFileFormat = SPSF_16kHz16BitMono;
			//SPSTREAMFORMAT iOutputFileFormat = SPSF_8kHz16BitMono;
			//��XWave�ɮצW��
			//wchar_t szOutputFileName[] = L"�ۧڤ���.wav";	
			//wchar_t szOutputFileName[] = L"�̶}�l���w���.wav";
			//wchar_t szOutputFileName[] = L"���i����ɭԪ��ۧڤ���.wav";
			wchar_t szOutputFileName[] = L"N07.wav";
		wchar_t *pszInputString = YourInputText_CString.AllocSysString();	// ��J����r��
		wchar_t *pszTagOutputString;				// ��X�����J�аO�y������r��
		int		iTagAdditionLen = 64;				// ���J�аO�y���ҷs�W���r�����

		pszTagOutputString = new wchar_t [wcslen(pszInputString)+iTagAdditionLen];

		swprintf(pszTagOutputString, L"<PitchModify para=\"%d;%d;%d\"/>%s", iLevel, iDirection, iVariance, pszInputString);

		TTSEngineTest(pszTagOutputString, szVoiceEngine,iSpeed, iMode, szOutputFileName, iOutputFileFormat);

		delete [] pszTagOutputString;
		

		 // pVoice->Release(); //6. ����TTS Engine (release the Text2Speech Engine after using)

		//pVoice = NULL;//7. �b����e set NULL(Set to NULL before Uninitialize the COM)

	}

	CoUninitialize(); // 8. �Ѱ� COM (Uninitialize the COM)

	return true;
}
void CSoundPlay::VolumnLower(void)
{
	float volumn;
	channel->getVolume(&volumn);

	volumn -= 0.8f;
	channel->setVolume(volumn);	
}

void CSoundPlay::VolumnHigher(void)
{
	float volumn;
	channel->getVolume(&volumn);

	volumn += 0.8f;
	channel->setVolume(volumn);
}
void CSoundPlay::ReadSignLanq(int numb)
{
char CharBuf [50];

for(int i=1;i<numb+1;i++)
	{
	  if(i<10)
	  sprintf (CharBuf, "SoundFiles/H0%d.wav", i);
	  else
	  sprintf (CharBuf, "SoundFiles/H%d.wav", i);
	
	  LoadSoundFile(CharBuf);

	  SignLanq.push_back(Sound1);
	}

}