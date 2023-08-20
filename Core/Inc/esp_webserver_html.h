/*
 * esp_webserver_html.h
 *
 *  Created on: 21 Mar 2023
 *      Author: dillo
 */

#ifndef INC_ESP_WEBSERVER_HTML_H_
#define INC_ESP_WEBSERVER_HTML_H_

static const char* html = "<html lang=\"en\"><head><meta charset=\"UTF-8\"><meta hhtp-equiv=\"X-UA-Compatible\" content=\"IE=edge\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>ESP-01S Client</title></head><body><H3>Click on the Button to toggle the LED</H3><p>Blue LED State<span id=\"BlueLed\"> </span></p><p>Red LED State<span id=\"RedLed\"> </span></p><button id = \"1\" onclick = \"ledToggle(this.id)\">Blue Led</button><button id = \"2\" onclick = \"ledToggle(this.id)\">Red Led</button><p>Green LED State<span id=\"GreenLed\"> __</span></p><button id = \"3\" onclick=\"disable(this.id)\">Update Continuously</button><button id = \"4\" onclick=\"enable(this.id)\" disabled>Stop Update</button><script>var id;function ledToggle(clicked_id){var xhr= new XMLHttpRequest();var url = clicked_id;xhr.open(\"GET\",url,true);xhr.send();xhr.onreadystatechange = function(){if(this.readyState == 4 && this.status == 200){if(this.responseText==\"10\"){document.getElementById(\"BlueLed\").innerHTML=\" OFF\";}else if(this.responseText==\"11\"){document.getElementById(\"BlueLed\").innerHTML=\" ON\";}else if(this.responseText==\"20\"){document.getElementById(\"RedLed\").innerHTML=\" OFF\";}else if(this.responseText==\"21\"){document.getElementById(\"RedLed\").innerHTML=\" ON\";}}};}function disable(clicked_id){id = setInterval(function interval(){var xhr= new XMLHttpRequest();var url = clicked_id;xhr.open(\"GET\",url,true);xhr.send();xhr.onreadystatechange = function(){if(this.readyState == 4 && this.status == 200){if(this.responseText==\"30\"){document.getElementById(\"GreenLed\").innerHTML=\" OFF\";}else if(this.responseText==\"31\"){document.getElementById(\"GreenLed\").innerHTML=\" ON\";}}};document.getElementById(\"3\").disabled = true;document.getElementById(\"4\").disabled = false;}, 3000);}function enable(clicked_id){clearInterval(id);document.getElementById(\"3\").disabled = false;document.getElementById(\"4\").disabled = true;}</script></body></html>";


#endif /* INC_ESP_WEBSERVER_HTML_H_ */
