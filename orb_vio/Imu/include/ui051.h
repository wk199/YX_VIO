#ifndef UI051_H
#define UI051_H

#include <pthread.h>

class UI051
{
public:
	UI051(void);
	~UI051(void);

public:
	bool InitPort();
	bool InitPort();
	bool OpenListenThread();
	bool CloseListenThread();
	bool WriteBytes(unsigned char* pData, unsigned int length);
	bool ReadBytes(char &cRead, int size);
		
private:
	bool openPort();
	bool openPort(unsigned char n_port);
	void closePort();
	static pthread_t ListenThread(void *pParam);

private:
	int n_port;
	static bool s_bExit;
	volatile HANDLE m_hListenThread;
	pthread_mutex_t m_csCommunicationSync;
};

#endif
