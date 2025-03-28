# Raspberry Pi 5 pinout

![Main Image](https://cdn.shopify.com/s/files/1/0195/1344/2404/files/7_5_5c4a976d-c917-4049-ba8f-85bb33d52568_1024x1024.png?v=1721153056)


<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Install raspi-config</title>
  <style>
    body {
      font-family: sans-serif;
      line-height: 1.6;
      margin: 2rem;
    }

    .code-block {
      position: relative;
      background-color: #f9f7f4;
      border-radius: 8px;
      padding: 1rem;
      margin-bottom: 1rem;
      font-family: monospace;
      white-space: pre;
      overflow-x: auto;
    }

    .copy-btn {
      position: absolute;
      top: 10px;
      right: 10px;
      border: none;
      background: transparent;
      cursor: pointer;
      font-size: 16px;
    }

    .code-block code .red {
      color: #d14;
      font-weight: bold;
    }

    h1 {
      font-size: 1.5rem;
      margin-bottom: 1rem;
    }

    small {
      color: gray;
    }
  </style>
</head>
<body>

<h1>install raspi-config</h1>

<div class="code-block">
  <button class="copy-btn" onclick="copyCode('code1')">📋</button>
  <code id="code1">
<span class="red">sudo</span> <span class="red">apt-get</span> update
<span class="red">sudo</span> <span class="red">apt-get</span> <span class="red">install</span> raspi-config
<span class="red">sudo</span> raspi-config
  </code>
</div>

<small>config to enable I2C / Serial port / One wire</small>

<div class="code-block">
  <button class="copy-btn" onclick="copyCode('code2')">📋</button>
  <code id="code2">
<span class="red">sudo</span> <span class="red">apt</span> <span class="red">install</span> i2c-tools
  </code>
</div>

<p><strong>recommand python V. 3.11</strong></p>

<script>
function copyCode(id) {
  const text = document.getElementById(id).innerText;
  navigator.clipboard.writeText(text).then(() => {
    alert("Copied to clipboard!");
  });
}
</script>

</body>
</html>
