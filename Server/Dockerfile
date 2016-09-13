FROM ubuntu:16.04
MAINTAINER vysakhpillai@gmail.com

RUN apt-get update && apt-get install -y apache2 supervisor mosquitto-clients mosquitto build-essential  
RUN mkdir -p /var/lock/apache2 /var/run/apache2 /var/run/sshd /var/log/supervisor /var/lock/mosquitto /var/run/mosquitto

#apis
RUN mkdir -p /var/www/api/listen
COPY configs/listen.sh /var/www/api/listen/to
RUN chmod +x /var/www/api/listen/to

RUN mkdir -p /var/www/api/mdtweet
COPY  configs/sendMessage.c /var/www/api/mdtweet
RUN gcc /var/www/api/mdtweet/sendMessage.c -o /var/www/api/mdtweet/for
run rm /var/www/api/mdtweet/sendMessage.c

#apache
COPY configs/serve-cgi-bin.conf /etc/apache2/conf-available/serve-cgi-bin.conf
COPY configs/serve-cgi-bin.conf /etc/apache2/conf-enabled/serve-cgi-bin.conf
COPY configs/apache2.conf /etc/apache2/apache2.conf
RUN mkdir -p /var/www/html
RUN rm -rf /var/www/html/*
ADD html /var/www/html 
RUN a2enmod cgi

COPY configs/mosquitto.conf /etc/mosquitto/mosquitto.conf
COPY configs/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

EXPOSE 80 443
CMD ["/usr/bin/supervisord"]
