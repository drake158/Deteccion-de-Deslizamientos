# Deteccion de deslizamientos 
## Resumen general
En el siguiente repositorio se describe las etapas de diseño, desarrollo, implementación y validación que fueron llevadas a cabo para desarrollar un prototipo capaz de detectar señales de inestabilidad o taludes en el suelo y alertar en tiempo real
## Motivación
Durante la temporada de lluvias en Colombia se registran mas de 600 municipios que presentan alto riego de presentar deslizamientos de tierra y cada año se reportan múltiples emergencias por este fenómeno dejando víctimas fatales.
Para reducir el numero de victimas causadas se requiere un sistema que permita predecir con anterioridad cuando va a pasar este fenómeno natural y poder evacuar a las personas.
Este debe tener algunas características como ser resistente al agua, consumir poca energía de una batería y no necesitar conexión a internet constantemente ya que su uso va principalmente a áreas rurales donde la conexión a internet y la electricidad no esta asegurada, por estas condiciones se denota que utilizando IoT se puede abarcar esta problemática de manera eficiente.
## Justificación
La solución alcanzada se justifica de manera que la solución propuesta es capaz de detectar pequeños movimientos o crecimiento en la inclinación del suelo además de características como humedad de la tierra que son factores determinantes de los deslizamientos y genera una alerta visual y sonora para que las personas que estén en el área evacuen con antelación.
También no es necesaria su conexión a internet para su uso ya que los datos obtenidos desde el dispositivo pueden ser leídos en el sitio sin necesidad de pasar por un servidor en la nube lo que lo hace efectivo en áreas rurales.
## Estructura de la documentación

# Solución propuesta
Nuestra solución consiste en un microcontrolador que junto con sensor acelerómetro y giroscopio, un detector de lluvia y un medidor de humedad en el suelo para poder detectar con antelación los desplazamientos y una alarma con luz led para que las personas cercanas sepan que deben evacuar.
## Restricciones de diseño
En la parte económica se identificaron restricciones referentes a los sensores utilizados, aunque eran adecuados para satisfacer las condiciones, la solución podría utilizar algunos más precisos para mejorar la detección y poder dar alarma de manera mas temprana y evitar activaciones de alarmas innecesarias.
