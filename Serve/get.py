import requests
data={'IMAGE':'C:/Users/VCC/Desktop/25.jpg'}
files={'IMAGE':('25.jpg',open('C:/Users/VCC/Desktop/25.jpg','rb'),'image/png',{})}
url = 'http://172.31.76.30/ThinkingQ/'
response = requests.request("POST",url=url,files=files)
html1 = response.text
print(html1)