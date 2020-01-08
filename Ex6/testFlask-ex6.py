
from flask import Flask,request,jsonify,make_response
import json

app = Flask(__name__)
# print(app.config)

# global variables
data_count2=0
adc_list2=[]
index_list2=[]
list_max2 = 10*100

@app.route("/")
def hello():
    # this is test page
    return "Hello World !!"

@app.route("/getadclist",methods=['GET'])
def getadclist():
     #--------------------------------------------
     global data_count2
     global index_list2
     global adc_list2
     global list_max2
     #--------------------------------------------
     # print( 'Request args = {}'.format(request.args) )
     json_rcv_str=request.args.get('JSON',type=str)
     # print('json_rcv_str:',  json_rcv_str )
     #
     json_rcv_dict=json.loads(json_rcv_str)
     # print("json_rcv_dict = {}".format( json_rcv_dict ))
     adc_rcv_list=json_rcv_dict['adc']
     # print(adc_rcv_list)
     print("adc rcv 0  :", adc_rcv_list[0])
     # print(type(adc_rcv_list[0]))
     #count_rcv=json_rcv_dict['count']
     #print("count=", count)
     #--------------------------------------------
     rcv_size=len(adc_rcv_list)
     del_len=len(adc_list2)+rcv_size-list_max2
     if del_len>0:
         # delete original list elements
         index_list2=index_list2[del_len:]
         adc_list2=adc_list2[del_len:]

     index_list2.extend(list(range(data_count2,data_count2+rcv_size)))
     data_count2=data_count2+rcv_size
     print("rcv_size=", rcv_size, "data_count=", data_count2)
     adc_list2.extend(adc_rcv_list)
     #--------------------------------------------
     return jsonify( data_count=data_count2, ret='OK' )

@app.route('/graph2')
def graph2():
  import math
  import numpy
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot
  from matplotlib.backends.backend_agg import FigureCanvasAgg
  import io

  global data_count2
  global index_list2
  global adc_list2
  fig = matplotlib.pyplot.figure()
  ax = fig.add_subplot(111)
  ax.plot(index_list2, adc_list2)

  canvas = FigureCanvasAgg(fig)
  buf = io.BytesIO()
  canvas.print_png(buf)
  data = buf.getvalue()

  response = make_response(data)
  response.headers['Content-Type'] = 'image/png'
  response.headers['Content-Length'] = len(data)
  return response


if __name__ == "__main__":
     app.run(host="0.0.0.0", port=50000, debug=True, threaded=True)
