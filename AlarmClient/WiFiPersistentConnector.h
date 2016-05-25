#ifndef WiFiPersistentConnector_h
#define WiFiPersistentConnector_h

class WiFiPersistentConnector {
private:
	const char *_ssid;
	const char *_pass;
	bool _isStarted;

public:
	WiFiPersistentConnector(const char *ssid, const char *pass);
	bool start();
	void stop();
	bool update();
};

#endif