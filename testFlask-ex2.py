
from flask import Flask,request,jsonify,make_response

app = Flask(__name__)
# print(app.config)

# global variables
data_count=0
adc_list=[]
index_list=[]
list_max = 100
freq = 200.0

@app.route("/")
def hello():
    # this is test page
    return "Hello World !!"

@app.route("/input_freq",methods=['GET','POST'])
def input_freq():
    global freq
    if request.method == "GET":
        freq = 200.0
        print ("Freq = ", freq)
        return """
           Current Freq is {} Hz (default) <br>
           <form action="/input_freq" method="POST">
           <label>Frequency  <input name="freq"></label>
           <input type="submit" value="submit">
           </form>""".format(str(freq))
    else:
        freq = request.form['freq']
        print ("Freq = ", freq)
        return """
           Current Freq is {} Hz<br>
           <form action="/input_freq" method="POST">
           <label>Frequency  <input name="freq"></label><br>
           <input type="submit" value="submit">
           </form>""".format(str(request.form['freq']))

@app.route("/getadc",methods=['GET'])
def getadc():
     #--------------------------------------------
     # print( 'Request = {}'.format(request.args) )
     #
     global data_count
     global index_list
     global adc_list
     global list_max
     global freq
     adc_value=request.args.get('ADC', type=int)
     #    print( "ADC = ", adc_value)
     print('Count = ', data_count )
     #
     if len(index_list) > list_max: 
        index_list.pop(0)
        adc_list.pop(0)
     #
     index_list.append(data_count)
     data_count=data_count+1
     adc_list.append(adc_value)
     #
     # print( 'index list = {}'.format( index_list ) )
     # print( 'adc   list = {}'.format( adc_list ) )
     #--------------------------------------------
     freq = float(freq) 
     return jsonify( adc=adc_value, freq=freq )
     #return jsonify( { 'adc':adc_value })

@app.route('/graph1')
def graph1():
  import math
  import numpy
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot
  from matplotlib.backends.backend_agg import FigureCanvasAgg
  import io

  global data_count
  global index_list
  global adc_list
  fig = matplotlib.pyplot.figure()
  ax = fig.add_subplot(111)
  ax.plot(index_list, adc_list)

  canvas = FigureCanvasAgg(fig)
  buf = io.BytesIO()
  canvas.print_png(buf)
  data = buf.getvalue()

  response = make_response(data)
  response.headers['Content-Type'] = 'image/png'
  response.headers['Content-Length'] = len(data)
  return response

if __name__ == "__main__":
     app.run(host="0.0.0.0", port=5000, debug=True, threaded=True)
