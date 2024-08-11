from flask import Flask, request

app = Flask(__name__)

@app.route('/log')
def handle():
    # print('request came')
    msg = request.args.get('msg')
    # print(msg)
    if msg:
        with open('log.txt', 'a') as f:
            f.write(msg)
            f.write('\n')
    return 'ok', 200

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')