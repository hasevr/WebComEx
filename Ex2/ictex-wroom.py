from flask import Flask,request,jsonify;
app = Flask(__name__)

@app.route("/")
def hello():
    return "Hello World !!"

@app.route("/getadc",methods=['GET'])
def getadc():
    print( "Request = {}".format( request.args ) )
    adc_value=request.args.get('ADC', type=int)
    print( "ADC = ", adc_value)
    return jsonify( adc=adc_value)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=50000, debug=True)