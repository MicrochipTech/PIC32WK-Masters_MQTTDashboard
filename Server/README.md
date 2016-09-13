This project is a dockerised version of the vppillai/HTTP-MQTTWrapper project. This is meant for one stop config and deploy .

I use Digitalocean for development and the following steps are for their x86_64 ubuntu16 droplet. But the steps will hold good for most systems. 

## here are the quick steps for the impatient

Run the following in your droplet to get teh server up and running.

```bash
apt-get update
apt-get upgrade -y
apt-get install git docker.io -y
git clone https://github.com/MicrochipTech/PIC32WK-Masters_MQTTDashboard.git
cd Server/masters-Docker
docker build -t myiotdashboard .
docker run -d -p 80:80 -p 443:443  --name iot_dashboard myiotdashboard
```

## Building and deploying this docker

This is a semi-detailed explanation of the steps above. 


On a vanilla droplet, run

```bash
apt-get update
apt-get upgrade -y
apt-get install git docker.io -y
```

Once Git is installed, clone this repo and from within the repo, issue the following command to pull all required images and create a docker image by the name 'myiotdashboard'. 

```bash
docker build -t myiotdashboard .
```

In the super fast network of DO, it takes around 5 minutes to create this image on a single CPU droplet. Once the image is created, issue the following command to run the Docker image and expose the ports via teh standard ports of the droplet. 

```bash
docker run -d -p 80:80 -p 443:443  --name iot_dashboard myiotdashboard 
```

***Note*** : In case there is an error that port 80 is in use, then issue the following command to stop the default apache instance. 

```bash
service apache2 stop
```

Finally, if you need to attach and access the docker via a shell, execute the following command. 

```bash
docker exec -i -t iot_dashboard /bin/bash
```


